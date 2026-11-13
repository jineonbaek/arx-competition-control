#!/usr/bin/env python3
"""Synthetic isolated IMU stream verifies fail-closed guard without touching hardware."""
from pathlib import Path
import os
import signal
import subprocess
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

root = Path(__file__).resolve().parents[1]
log = root/'reports/imu-gap-guard-test.log'
cmd = ['ros2', 'run', 'fast_lio', 'fastlio_mapping', '--ros-args',
       '--params-file', str(root/'config/mapping_test/fastlio_arm.yaml'),
       '-r', '__node:=isolated_imu_guard_test', '-r', '__ns:=/guard_test',
       '-p', 'common.imu_topic:=/guard_test/imu', '-p', 'common.lid_topic:=/guard_test/lidar',
       '-p', 'common.max_imu_gap_sec:=1.0', '-p', 'pcd_save.pcd_save_en:=false',
       '-p', 'publish.map_en:=false', '-p', 'frames.odom:=guard_odom', '-p', 'frames.imu:=guard_imu',
       '-r', '/tf:=/guard_test/tf']
for topic in ('cloud_registered', 'cloud_registered_body', 'cloud_effected', 'Laser_map', 'Odometry', 'path'):
    cmd += ['-r', f'/{topic}:=/guard_test/{topic}']
rclpy.init()
node = Node('isolated_guard_input')
pub = node.create_publisher(Imu, '/guard_test/imu', 10)
with log.open('w') as f:
    child = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        end = time.monotonic()+8
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        assert pub.get_subscription_count() > 0, 'Test subscription not discovered'
        for ns in [100_000_000_000, 100_005_000_000, 102_000_000_000]:
            m = Imu()
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            m.linear_acceleration.z = 9.81
            pub.publish(m)
            time.sleep(.1)
        code = child.wait(timeout=8)
        assert 'IMU discontinuity: stopping LIO' in log.read_text(), 'Guard did not trigger'
        result = {'passed': True, 'process_exit_code': code, 'synthetic_imu_gap_s': 1.995,
                  'robot_commands_sent': False, 'live_sensor_processes_untouched': True}
        (root/'reports/imu-gap-guard-test.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=5)
node.destroy_node()
rclpy.shutdown()
