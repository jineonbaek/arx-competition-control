"""Measure one bounded SDK pulse against FAST-LIO under live footprint checks."""
import argparse
import fcntl
import json
import time
import numpy as np
import rclpy
from axis_trial import AxisTrial, body_delta
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

axes = {'forward': (1,0,0), 'backward': (-1,0,0), 'left': (0,1,0),
        'right': (0,-1,0), 'ccw': (0,0,1), 'cw': (0,0,-1)}
p = argparse.ArgumentParser()
p.add_argument('axis', choices=axes)
p.add_argument('level', type=float)
args = p.parse_args()
direction = np.array(axes[args.axis])
limit = .065 if direction[2] else .28
if not 0 < args.level <= limit:
    p.error(f'level must be positive and at most {limit}')
rclpy.init(args=['--ros-args','-r','__node:=arx_mppi_sdk_calibration',
                 '-r','/pb/chassis_preview:=/arx_mppi_trial/calibration_preview'])
node = None
try:
    with open('/tmp/arx-pb-goal-drive.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        node = AxisTrial()
        node.destroy_timer(next(iter(node.timers)))
        odometry = []
        def record(msg):
            stamp = msg.header.stamp
            v = msg.twist.twist
            pose = msg.pose.pose
            odometry.append([time.monotonic(), stamp.sec + stamp.nanosec*1e-9,
                             pose.position.x, pose.position.y, pose.orientation.z,
                             pose.orientation.w, v.linear.x, v.linear.y, v.angular.z])
        node.create_subscription(Odometry, '/localization/odometry', record,
                                 qos_profile_sensor_data)
        node.transport.sdk_units = True
        node.transport.native_sdk = True
        node.spin(3.)
        with node.transport.client.open(node.transport.origin+'/api/state',timeout=.3) as reply:
            state=json.load(reply)
        if not (state['execution_enabled'] and state['nav_sdk_ready'] and
                state['body']['fresh'] and not state['chassis_active'] and
                not state['capabilities']['chassis']):
            raise RuntimeError('Console is busy or motion is disabled')
        # Wait for discovery and fresh samples before any command is sent.
        deadline = time.monotonic() + 6.
        while node.stale_inputs() and time.monotonic() < deadline:
            node.spin(.08)
        if stale := node.stale_inputs():
            raise RuntimeError('Calibration inputs stale: ' + ','.join(stale))
        def checked_pose():
            try:
                return node.guarded_pose(direction)
            except RuntimeError as exc:
                details = {}
                for frame in ('map','odom'):
                    if frame not in node.grids:
                        continue
                    pose = node.pose(frame,node.now())
                    c,s = np.cos(pose[2]),np.sin(pose[2])
                    x,y,w = direction
                    end = pose+[.09*(c*x-s*y),.09*(s*x+c*y),.20*w]
                    details[frame] = node.grids[frame][1].along([pose,end])['first']
                raise RuntimeError(f'{exc}; stale={node.stale_inputs()}; footprints={details}') from exc
        start = checked_pose()
        before=time.monotonic()
        history=[]
        sent=[]
        try:
            while time.monotonic()-before < .60:
                dt=time.monotonic()-before
                current=checked_pose()
                history.append([dt,*body_delta(start,current)])
                if np.linalg.norm(history[-1][1:3])>.018 or abs(history[-1][3])>.035:
                    break
                node.active=True
                node.transport.send(direction*args.level)
                sent.append(time.monotonic()-before)
                node.spin(.060)
        finally:
            brake_at = time.monotonic()-before
            node.brake()
        node.spin(1.1)
        settled=node.pose('odom',node.now())
        node.spin(.35)
        final=node.pose('odom',node.now())
        print(json.dumps(dict(axis=args.axis,sdk_level=args.level,samples=history,
            delta=body_delta(start,final).tolist(),settled_drift=body_delta(settled,final).tolist(),
            start_wall=before, sent=sent, brake_at=brake_at, odometry=odometry)))
finally:
    if node is not None:
        node.destroy_node()
    rclpy.try_shutdown()
