#!/usr/bin/env python3
"""Synthetic IMU gap regression in ROS domain 197.

Pass the staged executable, parameter file, and output log as arguments.
"""
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

binary, config, log_path = (Path(arg).resolve() for arg in sys.argv[1:])
rclpy.init(domain_id=197)
node = rclpy.create_node('arx_imu_gap_regression')
imu_pub = node.create_publisher(Imu, '/imu_gap_test/imu', 1000)
lidar_pub = node.create_publisher(CustomMsg, '/imu_gap_test/lidar', 20)
odom = []
node.create_subscription(Odometry, '/imu_gap_test/odom', lambda m: odom.append(m), 20)

cmd = [str(binary), '--ros-args', '--params-file', str(config),
       '-r', '__node:=arx_imu_gap_lio', '-r', '__ns:=/imu_gap_test',
       '-p', 'common.imu_topic:=/imu_gap_test/imu', '-p', 'common.lid_topic:=/imu_gap_test/lidar',
       '-p', 'common.max_imu_gap_sec:=0.1', '-p', 'pcd_save.pcd_save_en:=false',
       '-p', 'publish.map_en:=false', '-p', 'point_filter_num:=1',
       '-r', '/Odometry:=/imu_gap_test/odom', '-r', '/tf:=/imu_gap_test/tf']
for topic in ('cloud_registered', 'cloud_registered_body', 'cloud_effected', 'Laser_map', 'path'):
    cmd += ['-r', f'/{topic}:=/imu_gap_test/{topic}']

def wait_until(check, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        if check():
            return True
    return False

def imu(stamp):
    m=Imu()
    m.header.stamp.sec, m.header.stamp.nanosec=divmod(stamp,1_000_000_000)
    m.linear_acceleration.z=9.81
    imu_pub.publish(m)

points=[]
for i in range(1,16):
    for j in range(-8,9):
        a=i*.21
        b=j*.21
        points.extend([(3.5,a,b),(a,3.2,b),(a,b,-1.4)])

cloud = CustomMsg()
cloud.points = [CustomPoint(x=x,y=y,z=z,offset_time=i*90_000_000//(len(points)-1),
                            reflectivity=50,line=i%4) for i,(x,y,z) in enumerate(points)]
cloud.point_num=len(points)

def scan(stamp):
    cloud.header.stamp.sec, cloud.header.stamp.nanosec=divmod(stamp,1_000_000_000)
    cloud.timebase=stamp
    lidar_pub.publish(cloud)

with log_path.open('w') as log:
    child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,
                           env=dict(os.environ, ROS_DOMAIN_ID='197'))
    try:
        assert wait_until(lambda: imu_pub.get_subscription_count() and lidar_pub.get_subscription_count(), 9), log_path.read_text()
        stamp=100_000_000_000
        def frames(n):
            global stamp
            for _ in range(n):
                for i in range(20):
                    imu(stamp+i*5_000_000)
                scan(stamp)
                stamp+=100_000_000
                wait_until(lambda: child.poll() is not None,.12)
        frames(18)
        assert len(odom)>=4, 'Baseline must publish odometry: '+log_path.read_text()
        # Simulate a 250 ms forward gap without fabricating missing data.
        before=len(odom)
        scan(stamp)
        stamp+=245_000_000
        frames(30)
        assert child.poll() is None, 'FAST-LIO exited on a 250 ms IMU gap'
        assert len(odom)>=before+3, 'Odometry did not recover after clean IMU and scans'
        assert any('IMU' in line and 'recover' in line.lower() for line in log_path.read_text().splitlines())
        for msg in odom[before:]:
            p=msg.pose.pose.position
            assert math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z) < .04, 'Stationary pose jumped'
        before=len(odom)
        imu(stamp-5_000_000)
        imu(stamp-10_000_000)
        frames(8)
        assert child.poll() is None and len(odom)>before, 'Duplicate or small reorder stalled FAST-LIO'
        before=len(odom)
        stamp+=2_000_000_000
        frames(12)
        assert child.poll() is None, 'Prolonged IMU gap killed the node'
        assert len(odom)==before, 'Prolonged gap published unverified odometry'
        print('Passed: short-gap pose recovery, duplicate/reordered input, and prolonged-gap hold')
    finally:
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=8)
        node.destroy_node()
        rclpy.try_shutdown()

