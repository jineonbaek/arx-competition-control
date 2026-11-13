#!/usr/bin/env python3
import argparse
from pathlib import Path
import time
import json
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg

p = argparse.ArgumentParser()
p.add_argument('--duration', type=float, default=20)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
rclpy.init()
node = Node('check_dual_lidar')
rows = {n: [] for n in ('arm', 'top')}
counts = {}
stamps = {}
last = {}
subs = []
def receive(name, kind, msg):
    key = name+'/'+kind
    counts[key] = counts.get(key, 0)+1
    stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    stamps.setdefault(key, []).append(stamp)
    last[key] = time.monotonic()
    if kind == 'odometry':
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        rows[name].append([stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                           t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z])
        assert msg.header.frame_id == name+'_lidar_odom'
        assert msg.child_frame_id == name+'_lio_imu'
        assert np.isfinite(msg.pose.covariance).all()
        assert np.isfinite(msg.twist.covariance).all()
        for covariance in (msg.pose.covariance, msg.twist.covariance):
            matrix = np.asarray(covariance).reshape(6, 6)
            assert np.allclose(matrix, matrix.T, atol=1e-8)
            assert np.linalg.eigvalsh(matrix).min() >= -1e-8


for n in rows:
    for kind, cls in [('odometry', Odometry), ('imu', Imu), ('lidar', CustomMsg)]:
        subs.append(node.create_subscription(cls, f'/dual/{n}/{kind}', lambda m, n=n, k=kind: receive(n, k, m), 100))
begin = time.monotonic()
while time.monotonic()-begin < a.duration:
    rclpy.spin_once(node, timeout_sec=.1)
result = {'duration_s': time.monotonic()-begin, 'robot_commands_sent': False, 'sources': {}}
for name, data in rows.items():
    v = np.asarray(data)
    assert len(v) > 50 and np.isfinite(v).all()
    out = {'samples': len(v), 'position_span_xyz_m': np.ptp(v[:, 1:4], axis=0).tolist(),
           'mean_linear_velocity_imu_m_s': v[:, 8:11].mean(axis=0).tolist(),
           'rms_linear_speed_m_s': float(np.sqrt(np.mean(np.sum(v[:, 8:11]**2, axis=1)))),
           'rms_angular_speed_rad_s': float(np.sqrt(np.mean(np.sum(v[:, 11:14]**2, axis=1)))),
           'twist_not_all_zero': bool(np.any(v[:, 8:14] != 0)), 'topics': {}}
    for kind in ('odometry', 'imu', 'lidar'):
        key = name+'/'+kind
        times = np.array(stamps[key])
        diffs = np.diff(times)
        out['topics'][kind] = {'count': len(times), 'hz': float((len(times)-1)/(times[-1]-times[0])),
                               'max_header_gap_s': float(diffs.max()), 'nonincreasing_stamps': int((diffs <= 0).sum()),
                               'last_age_s': time.monotonic()-last[key]}
    result['sources'][name] = out
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
