"""Execute only fresh interactive RViz arrows using the commissioned low-speed axes."""
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from std_msgs.msg import Bool, Empty, String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from waypoint_controls import Waypoints
from axis_trial import AxisTrial, body_delta
from follow_preview import angle
from path_audit import planar
from continuous_track import OffPath, CruiseLimiter, PERIOD, POSITION_TOL, YAW_TOL, tracking_command, swept_poses


def stamp(s):
    return s.sec + s.nanosec * 1e-9


class Driver(AxisTrial):
    def __init__(self):
        super().__init__()
        self.intent=self.create_publisher(Bool,"/pb/head_motion_intent",1)
        self.transport.sdk_units=True
        self.require_bottom_mark=os.getenv('ROS_DOMAIN_ID')=='192'
        self.bottom_mark_stamp=0.
        if self.require_bottom_mark:
            self.create_subscription(PointCloud2,'/pb/bottom/nav_mark',lambda m:setattr(self,'bottom_mark_stamp',stamp(m.header.stamp)),1)
        run = os.getenv('ARX_ADAPT_RUN')
        self.ready_file = Path(run) / 'drive-ready' if run and os.getenv('ARX_DESKTOP_MANAGED') == '1' else None
        self.watch_slam = os.getenv('ARX_DESKTOP_MODE') == 'mapping-navigation'
        self.dual_origin_health = None
        self.require_dual_origin = os.getenv('ARX_MAPPING_ENGINE') == 'dual-origin'
        if self.require_dual_origin:
            self.create_subscription(Bool, '/pb/dual_origin/ready', self.receive_dual_origin_health, 1)
        self.previous_alignment = None
        self.boot=self.now()
        self.target=self.route=None
        self.seq=self.seen=0
        self.state=self.create_publisher(String,'/pb/drive_status',10)
        self.finished=self.create_publisher(PoseStamped,"/pb/drive_finished",5)
        self.mark=self.create_publisher(Marker,'/pb/path_issue',5)
        self.waypoints=Waypoints(self)
        self.create_subscription(Empty,'/pb/drive_cancel',lambda _:self.cancel('cancelled'),5)

    def brake(self):
        was_moving=self.active
        super().brake()
        if was_moving:self.intent.publish(Bool(data=False))

    def status(self,value):
        msg=String(data=value);self.state.publish(msg);self.get_logger().info(value)

    def clear_marker(self):
        m=Marker();m.header.frame_id='map';m.ns='cone_trial_corridor';m.id=0;m.action=Marker.DELETE
        self.mark.publish(m)

    def cancel(self,why):
        self.brake()
        if self.target is not None:self.finished.publish(self.arrow)
        self.seq+=1;self.target=self.route=None
        self.status(why)
        self.waypoints.finish(why)

    def changed(self,_):
        if self.target is not None:self.cancel('cancelled_by_preview_point')

    def plan_changed(self,m):
        if self.waypoints.from_self(m):return
        self.waypoints.override()
        self.accept_goal(m)

    def accept_goal(self,m):
        if self.ready_file is not None and not self.ready_file.is_file():
            self.status('waiting_for_localization')
            return
        self.previous_alignment = None
        if self.target is not None:self.cancel('superseded')
        else:
            self.seq+=1
            self.route=None
            self.status('waiting_for_plan')
        t=stamp(m.header.stamp);q=m.pose.orientation
        goal=planar(m.pose)
        if (m.header.frame_id!='map' or t<self.boot or not 0<=self.now()-t<=2 or
            not np.isfinite(goal).all() or not abs(math.hypot(q.x,q.y,q.z,q.w)-1)<.002):
            self.status('ignored_invalid_or_old_goal');return
        self.target=goal;self.arrow=m;self.request=t;self.started=time.monotonic()

    def receive(self,m):
        if self.target is None:return
        previous=self.route
        self.route=None
        if not m.poses:
            if previous is not None:self.cancel('path_withdrawn')
            return
        t=stamp(m.header.stamp)
        if m.header.frame_id!='map' or t<self.request or not 0<=self.now()-t<3:return
        route=np.array([planar(p.pose) for p in m.poses])
        if not np.isfinite(route).all() or np.linalg.norm(route[-1,:2]-self.target[:2])>=.06:
            return
        # Smac may finish at a lattice heading. Sweep from it to the arrow pose.
        terminal = np.vstack((route[-1], self.target))
        if 'map' not in self.grids:
            return
        if self.grids['map'][1].along(terminal)['first']:
            self.cancel('terminal_sweep_blocked')
            return
        route = np.vstack((route, self.target))
        if self.grids['map'][1].along(route)['first']:
            self.cancel('replan_blocked')
            return
        self.route=route;self.route_time=t

    def receive_dual_origin_health(self, msg):
        self.dual_origin_health = time.monotonic() if msg.data else None

    def checked(self):
        if self.require_dual_origin and (self.dual_origin_health is None or
                time.monotonic()-self.dual_origin_health > .8):
            raise ValueError('dual_origin_mapping_stale')
        if self.seq!=self.seen or self.target is None:raise ValueError('goal_changed')
        if self.require_bottom_mark and not 0<=self.now()-self.bottom_mark_stamp<1.:raise ValueError('bottom_obstacles_stale')
        if stale := self.stale_inputs():
            raise ValueError('sensor_or_costmap_stale ' + ','.join(stale))
        poses={f:self.pose(f,self.now()) for f in ('map','odom')}
        for f,p in poses.items():
            if self.grids[f][1].inspect(p)['first_cell']:raise ValueError(f+'_footprint_blocked')
        if self.grids['map'][1].inspect(self.target)['first_cell']:raise ValueError('goal_blocked')
        if self.watch_slam:
            # Compose map->base with inverse(odom->base) to isolate SLAM correction.
            yaw = angle(poses['map'][2] - poses['odom'][2])
            c, s = math.cos(yaw), math.sin(yaw)
            x, y = poses['odom'][:2]
            alignment = np.array([poses['map'][0] - c*x + s*y, poses['map'][1] - s*x - c*y, yaw])
            previous = self.previous_alignment
            self.previous_alignment = alignment
            if previous is not None and (np.linalg.norm(alignment[:2]-previous[:2]) > .20 or
                                         abs(angle(yaw-previous[2])) > .15):
                raise ValueError('slam_alignment_jump_resend_goal')
        return poses['map']

    def verify_arrival(self):
        self.brake()
        samples = []
        end = time.monotonic() + 1.
        while time.monotonic() < end:
            error = body_delta(self.checked(), self.target)
            if np.linalg.norm(error[:2]) > POSITION_TOL or abs(error[2]) > YAW_TOL:
                raise ValueError('outside_arrival_tolerance')
            samples.append(self.pose('odom', self.now()))
            self.spin(PERIOD)
        span = np.ptp(np.unwrap(np.asarray(samples), axis=0), axis=0)
        if np.linalg.norm(span[:2]) > .009 or span[2] > .02:
            raise ValueError('not_stationary')
        return 'arrived_stopped'

    def run_goal(self):
        previous = np.zeros(3)
        measured = np.zeros(3)
        limiter = CruiseLimiter()
        last_log = -math.inf
        response_time = time.monotonic()
        sample_pose = progress_pose = None
        recovery_start = None
        recovery_stamp = 0.
        scan_wait_started = None
        last_tick = time.monotonic() - PERIOD
        while time.monotonic() - self.started < 480:
            tick = time.monotonic()
            dt = tick - last_tick
            last_tick = tick
            if self.active and dt > .30:
                raise ValueError('control_cycle_overrun')
            # Brake on a short gap while waiting for a new scan, then recheck the whole path.
            stale = self.stale_inputs()
            if stale == ['/pb/scan']:
                if scan_wait_started is None:
                    scan_wait_started = tick
                    self.brake()
                    previous = np.zeros(3)
                    measured = np.zeros(3)
                    sample_pose = None
                    limiter = CruiseLimiter()
                    response_time = tick
                    self.status('waiting_for_fresh_scan')
                if tick - scan_wait_started <= .30:
                    self.spin(PERIOD)
                    continue
            pose = self.checked()
            if recovery_start is not None:
                if tick-recovery_start>12:raise ValueError('off_path_no_valid_replan')
                if self.route is None or self.route_time<=recovery_stamp:
                    self.spin(PERIOD)
                    continue
            if self.route is None or not 0 <= self.now() - self.route_time < 3:
                if self.active or tick - self.started > 10:
                    raise ValueError('no_current_path')
                self.spin(PERIOD)
                continue
            odom = self.pose('odom', self.now())
            if sample_pose is None:
                sample_pose, sample_time = odom, tick
                progress_pose, progress_time = odom, tick
            if tick - sample_time >= .30:
                measured = body_delta(sample_pose, odom) / (tick - sample_time)
                sample_pose, sample_time = odom, tick
                if np.linalg.norm(measured[:2]) > .012 or abs(measured[2]) > .03:
                    response_time = tick
            limiter.update(measured, previous, dt)
            try:
                cmd, arrived = tracking_command(self.route, pose, self.target, previous, dt, tick-response_time, limiter)
            except OffPath as exc:
                self.brake()
                if recovery_start is None:
                    recovery_start=tick
                    self.status(f'waiting_for_same_goal_replan {exc}')
                recovery_stamp=self.now()
                self.route=None
                previous=np.zeros(3)
                sample_pose=None
                response_time=tick
                limiter=CruiseLimiter()
                self.spin(PERIOD)
                continue
            if recovery_start is not None:
                self.status('following_replanned_path')
                recovery_start=None
            if arrived:
                return self.verify_arrival()
            if tick - progress_time >= 3:
                delta = body_delta(progress_pose, odom)
                if np.linalg.norm(delta[:2]) < .006 and abs(delta[2]) < .015:
                    raise ValueError('no_progress')
                progress_pose, progress_time = odom, tick
            for frame, current in [('map', pose), ('odom', odom)]:
                for sweep in swept_poses(current, cmd, measured):
                    issue = self.grids[frame][1].along(sweep)['first']
                    if issue:
                        raise ValueError(f'{frame}_sweep_blocked {issue}')
            elapsed = time.monotonic() - tick
            if stale := self.stale_inputs():
                if stale == ['/pb/scan'] and scan_wait_started is None:
                    self.brake()
                    previous = np.zeros(3)
                    measured = np.zeros(3)
                    sample_pose = None
                    limiter = CruiseLimiter()
                    response_time = tick
                    scan_wait_started = tick
                    self.status('waiting_for_fresh_scan')
                    self.spin(PERIOD)
                    continue
                raise ValueError(f'stale_after_sweep {",".join(stale)} cycle_ms={elapsed*1000:.1f}')
            if elapsed > .20:
                raise ValueError(f'control_sweep_overrun cycle_ms={elapsed*1000:.1f}')
            scan_wait_started = None
            self.intent.publish(Bool(data=True))
            self.active = True
            self.transport.send(cmd)
            if tick - last_log >= .5:
                self.get_logger().info(json.dumps(dict(pose=pose.tolist(),command=cmd.tolist(),xy_cap_sdk=limiter.xy,measured=measured.tolist(),cycle_ms=round(1000*dt,1))))
                last_log = tick
            previous = cmd
            self.spin(max(.001, PERIOD - (time.monotonic() - tick)))
        raise ValueError('goal_timeout')

    def serve(self):
        self.spin(.5);self.clear_marker();self.status('ready_for_new_rviz_arrow')
        while rclpy.ok():
            self.spin(.08)
            if self.target is None:continue
            self.seen=self.seq
            try:
                with self.transport.client.open(self.transport.origin+"/api/state",timeout=.3) as res:console=json.load(res)
                if (not console["execution_enabled"] or not console["body"]["fresh"] or console["chassis_active"] or console["capabilities"]["chassis"]):raise ValueError("console_not_ready_or_busy")
                self.transport.native_sdk = bool(console.get("nav_sdk_ready", False))
                self.status("following_new_arrow")
                result=self.run_goal()
            except Exception as e:result='stopped: '+str(e)
            finally:self.brake()
            if self.seen==self.seq:self.cancel(result)


if __name__=='__main__':
    if os.getenv('ROS_DOMAIN_ID')!='192' or os.getenv('ARX_REPLAY','0')=='1':
        raise SystemExit('Live NUC navigation domain required')
    import fcntl
    lock=open('/tmp/arx-pb-goal-drive.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init();n=Driver()
    try:n.serve()
    finally:n.brake();n.destroy_node();rclpy.try_shutdown()
