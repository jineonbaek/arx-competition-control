#!/usr/bin/env python3
"""Synthetic integration test in isolated ROS domain 197, no CAN or motor SDK."""
import os
if os.environ.get('ROS_DOMAIN_ID')!='197':raise SystemExit('Run this test ONLY with ROS_DOMAIN_ID=197')
from pathlib import Path
import signal
import subprocess
import time
import json
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

root=Path(__file__).resolve().parents[1];report=root/'reports/wheel-imu-fusion';report.mkdir(parents=True,exist_ok=True)
children=[];handles=[]
for filename,cmd in [('ekf-synthetic.log',['ros2','run','robot_localization','ekf_node','--ros-args','--params-file',str(root/'config/wheel_imu_fusion/ekf.yaml'),'-r','__node:=ekf_filter_node','-r','odometry/filtered:=/fusion/odometry_internal']),('gate-synthetic.log',['/usr/bin/python3',str(root/'scripts/fused_odom_gate.py')])]:
    f=(report/filename).open('w');handles.append(f);children.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
rclpy.init();n=Node('synthetic_wheel_dual_imu_inputs');wheel=n.create_publisher(Odometry,'/wheel/odometry',20);imus=[n.create_publisher(Imu,'/fusion/imu/'+name+'_yaw_rate',100) for name in ['arm','top']];received=[]
def output(m):received.append([time.monotonic(),m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z,m.pose.pose.position.x,m.pose.pose.position.y])
sub=n.create_subscription(Odometry,'/odometry/filtered',output,100)
def publish(wheels=True):
    stamp=n.get_clock().now().to_msg()
    if wheels:
        m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom_wheel';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=.2;m.twist.twist.linear.y=.1;m.twist.twist.angular.z=.2
        m.twist.covariance[0]=.0004;m.twist.covariance[7]=.0004;m.twist.covariance[35]=.0004;wheel.publish(m)
    for p in imus:
        m=Imu();m.header.stamp=stamp;m.header.frame_id='base_link';m.orientation.w=1.;m.orientation_covariance[0]=-1.;m.linear_acceleration_covariance[0]=-1.;m.angular_velocity.z=.2;m.angular_velocity_covariance[0]=1e6;m.angular_velocity_covariance[4]=1e6;m.angular_velocity_covariance[8]=.0002;p.publish(m)
def run(seconds,wheels=True):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        publish(wheels);rclpy.spin_once(n,timeout_sec=.01)
try:
    end=time.monotonic()+8
    while (wheel.get_subscription_count()==0 or min(p.get_subscription_count() for p in imus)==0) and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    run(7)
    assert len(received)>100,'No valid fused odometry'
    last=received[-1];assert abs(last[1]-.2)<.02 and abs(last[2]-.1)<.02 and abs(last[3]-.2)<.02,last
    lost=time.monotonic();run(.7,False)
    assert not any(row[0]>lost+.4 for row in received),'Final output continued after wheel timeout'
    count=len(received);run(.7,True)
    assert len(received)==count,'Fault latch cleared automatically'
    result={'passed':True,'ros_domain':197,'hardware_inputs_used':False,'can_frames_sent':0,'fused_messages_before_timeout':count,'final_twist_before_timeout':last[1:4],'wheel_timeout_suppresses_output':True,'recovery_requires_restart':True}
    (report/'ekf-synthetic-result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
    n.destroy_node();rclpy.shutdown()
    for child in children:
        if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
    for child in children:
        try:child.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM)
    for f in handles:f.close()
