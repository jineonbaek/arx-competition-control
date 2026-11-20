"""Execute only fresh interactive RViz arrows using the commissioned low-speed axes."""
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped,PoseArray,PoseWithCovarianceStamped
from rclpy.qos import QoSProfile,DurabilityPolicy
from chassis_pose_policy import SnapshotLease,MOTION_MAX_AGE_NS
from recovery_budget import paused_budget
from chassis_pose import valid_pose,planar as authority_planar
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
        self.pose_snapshot=None;self.pose_lease=SnapshotLease(max_age_ns=MOTION_MAX_AGE_NS)
        self.create_subscription(PoseArray,'/pb/chassis_pose',self.receive_chassis_snapshot,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.relocalizing,10)
        self.boot=self.now()
        self.target=self.route=None
        self.seq=self.seen=0
        self.pending_output_ends=set()
        self.pending_route_finish=None
        self.pending_output_task=None
        self._retrying_output_ends=False
        self.state=self.create_publisher(String,'/pb/drive_status',10)
        self.finished=self.create_publisher(PoseStamped,"/pb/drive_finished",5)
        self.mark=self.create_publisher(Marker,'/pb/path_issue',5)
        self.waypoints=Waypoints(self)
        self.create_subscription(Empty,'/pb/drive_cancel',lambda _:self.cancel('cancelled'),5)
        self.output_end_timer=self.create_timer(.1,self.retry_output_ends)

    def brake(self):
        # Temporary input/replan hold. Remain in the same task generation.
        if self.active:
            self.transport.send((0.,0.,0.))
            self.active=False;self.intent.publish(Bool(data=False))

    def end_output(self,handle=None):
        handle=self.transport.current if handle is None else handle
        self.pending_output_ends.add(handle)
        try:self.transport.end_output(handle)
        except Exception as exc:
            self.get_logger().warning('Waiting for scoped output end confirmation: '+str(exc),throttle_duration_sec=3.)
            return False
        self.pending_output_ends.discard(handle)
        if handle==self.transport.current:
            self.active=False;self.intent.publish(Bool(data=False))
        return True

    def retry_output_ends(self):
        if self._retrying_output_ends:return not (self.pending_output_ends or self.pending_output_task)
        self._retrying_output_ends=True
        try:
            for handle in tuple(self.pending_output_ends):self.end_output(handle)
            if self.pending_output_ends:return False
            task=self.pending_output_task
            if task is not None:
                if task[0]==self.seq and self.target is not None:
                    if not self.end_output():return False
                    self.transport.begin_task(task_id=task[1])
                self.pending_output_task=None
            pending=self.pending_route_finish
            self.pending_route_finish=None
            if pending is not None and pending[0]==self.seq:
                self.waypoints.finish(pending[1])
            return not (self.pending_output_ends or self.pending_output_task)
        finally:self._retrying_output_ends=False

    @paused_budget
    def wait_output_ends(self,epoch):
        while rclpy.ok():
            if self.seq!=epoch or self.target is None:return False
            if self.retry_output_ends():return self.seq==epoch and self.target is not None
            self.spin(.1)
        return False

    def status(self,value):
        msg=String(data=value);self.state.publish(msg);self.get_logger().info(value)

    def clear_marker(self):
        m=Marker();m.header.frame_id='map';m.ns='cone_trial_corridor';m.id=0;m.action=Marker.DELETE
        self.mark.publish(m)

    def cancel(self,why):
        arrow=self.arrow if self.target is not None else None
        # Revoke intent before any transport operation. A failed stop must not
        # leave an executable old epoch that can revive when ROS reconnects.
        self.seq+=1;self.target=self.route=None
        self.pending_route_finish=None
        self.pending_output_task=None
        ended=self.end_output()
        if arrow is not None:self.finished.publish(arrow)
        self.status(why)
        if why=='arrived_stopped' and not ended:
            self.pending_route_finish=(self.seq,why)
        else:self.waypoints.finish(why)

    def changed(self,_):
        if self.target is not None:self.cancel('cancelled_by_preview_point')

    def plan_changed(self,m):
        if self.waypoints.from_self(m):return
        if self.goal_candidate(m) is None:
            self.status('ignored_invalid_or_old_goal');return
        self.waypoints.override()
        self.accept_goal(m)

    def goal_candidate(self,m):
        try:
            t=stamp(m.header.stamp);p=m.pose.position;q=m.pose.orientation
            if (m.header.frame_id!='map' or t<self.boot or not 0<=self.now()-t<=2 or
                    not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or
                    not abs(math.hypot(q.x,q.y,q.z,q.w)-1)<.002):return None
            goal=planar(m.pose)
            return (t,goal) if np.isfinite(goal).all() else None
        except (AttributeError,TypeError,ValueError,OverflowError):return None

    def accept_goal(self,m):
        candidate=self.goal_candidate(m)
        if candidate is None:
            self.status('ignored_invalid_or_old_goal');return
        if self.ready_file is not None and not self.ready_file.is_file():
            self.status('waiting_for_localization')
            return
        self.previous_alignment = None
        if self.target is not None:self.cancel('superseded')
        else:
            self.seq+=1
            self.route=None
            self.status('waiting_for_plan')
        t,goal=candidate
        self.target=goal;self.arrow=m;self.request=t;self.started=time.monotonic()
        self.pending_output_task=(self.seq,'goal-'+str(self.seq))
        if not self.retry_output_ends():self.status('previous_output_end_unconfirmed')

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

    def checked(self, *, occupancy=True):
        if self.require_dual_origin and (self.dual_origin_health is None or
                time.monotonic()-self.dual_origin_health > .8):
            raise ValueError('dual_origin_mapping_stale')
        if self.seq!=self.seen or self.target is None:raise ValueError('goal_changed')
        if self.require_bottom_mark and not 0<=self.now()-self.bottom_mark_stamp<1.:raise ValueError('bottom_obstacles_stale')
        if stale := self.stale_inputs():
            raise ValueError('sensor_or_costmap_stale ' + ','.join(stale))
        mp,op,_=self.motion_poses();poses={'map':mp,'odom':op}
        if occupancy:
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

    def motion_poses(self):
        # One source-stamped map/odom batch, with the shared 1s motion budget.
        # Temporary body-free map projection retains its independent 650ms lease.
        m=self.pose_snapshot;now=self.get_clock().now().nanoseconds
        if (m is None or not self.pose_lease.usable(now) or len(m.poses)!=2
                or not all(valid_pose(p) for p in m.poses)
                or not 0<=now-Time.from_msg(m.header.stamp).nanoseconds<=MOTION_MAX_AGE_NS):
            raise ValueError('motion_pose_stale_or_invalid')
        return np.array(authority_planar(m.poses[0])),np.array(authority_planar(m.poses[1])),stamp(m.header.stamp)

    def validate_motion_proof(self,source):
        # The batch used for all geometric proofs must still be authoritative
        # and within the shared1s motion bound at the actual send boundary.
        _,_,current=self.motion_poses()
        if current!=source:raise ValueError('motion_pose_changed_during_proof')

    def pose(self,frame,now):
        if frame not in ('map','odom'):raise ValueError('invalid_motion_frame')
        batch=self.motion_poses();return batch[0 if frame=='map' else 1]

    def special_stale_inputs(self):
        stale=[s for s in self.stale_inputs() if not s.endswith('_costmap')]
        if self.require_bottom_mark and not 0<=self.now()-self.bottom_mark_stamp<1.:stale.append('bottom_obstacles_stale')
        return stale

    def receive_chassis_snapshot(self,m):
        if (m.header.frame_id=='map' and len(m.poses)==2 and all(valid_pose(p) for p in m.poses)
                and self.pose_lease.offer(Time.from_msg(m.header.stamp).nanoseconds,self.get_clock().now().nanoseconds)):
            self.pose_snapshot=m
        else:self.pose_snapshot=None;self.pose_lease.withdraw()

    def relocalizing(self,m):
        if m.header.frame_id=='map':self.pose_snapshot=None;self.pose_lease.withdraw(self.get_clock().now().nanoseconds)

    def special_poses(self):
        if self.require_dual_origin and (self.dual_origin_health is None or time.monotonic()-self.dual_origin_health>.8):
            raise ValueError('dual_origin_mapping_stale')
        if stale:=self.special_stale_inputs():raise ValueError('special_sensor_stale '+','.join(stale))
        m=self.pose_snapshot
        if m is None or not self.pose_lease.usable(self.get_clock().now().nanoseconds):
            self.pose_snapshot=None;raise ValueError('special_pose_authority_invalid（定位尚未有效或正在重定位）')
        if not all(valid_pose(p) for p in m.poses):raise ValueError('special_invalid_pose')
        return np.array(authority_planar(m.poses[0])),np.array(authority_planar(m.poses[1])),stamp(m.header.stamp)

    def accept_special_goal(self,m):
        candidate=self.goal_candidate(m)
        if candidate is None:raise ValueError('ignored_invalid_or_old_goal')
        if self.target is not None or self.waypoints.blocking():raise ValueError('special_control_busy')
        if self.ready_file is not None and not self.ready_file.is_file():raise ValueError('execution_gate_closed')
        self.seq+=1
        self.request,self.target=candidate;self.arrow=m;self.started=time.monotonic();self.route=None
        self._special_execution=True;self.status('special_straight_selected')
        self.pending_output_task=(self.seq,'special-'+str(self.seq))
        if not self.retry_output_ends():self.status('previous_output_end_unconfirmed')

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

    def prepare_execution(self):
        with self.transport.client.open(self.transport.origin+"/api/state",timeout=.3) as res:console=json.load(res)
        if (not console["execution_enabled"] or not console["body"]["fresh"] or console["capabilities"]["chassis"]):raise ValueError("console_not_ready_or_busy")
        self.transport.native_sdk=bool(console.get("nav_sdk_ready",False))

    def serve(self):
        self.spin(.5);self.clear_marker();self.status('ready_for_new_rviz_arrow')
        while rclpy.ok():
            self.spin(.08)
            if self.target is None:continue
            if not self.wait_output_ends(self.seq):continue
            self.seen=self.seq
            output_handle=self.transport.current
            try:
                self.prepare_execution()
                self.status("following_new_arrow")
                with self.transport.task_scope(output_handle):result=self.run_goal()
            except Exception as e:result='stopped: '+str(e)
            finally:
                self.end_output(output_handle)
                if self.seen==self.seq and self.transport.current!=output_handle:self.end_output()
            if self.seen==self.seq:self.cancel(result)


if __name__=='__main__':
    if os.getenv('ROS_DOMAIN_ID')!='192' or os.getenv('ARX_REPLAY','0')=='1':
        raise SystemExit('Live NUC navigation domain required')
    import fcntl
    lock=open('/tmp/arx-pb-goal-drive.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init();n=Driver()
    try:n.serve()
    finally:n.end_output();n.destroy_node();rclpy.try_shutdown()
