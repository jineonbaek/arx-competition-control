#!/usr/bin/env python3
"""Final odometry only when wheel and both IMUs are fresh. No control output."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster


class Freshness:
    def __init__(self,timeout=.25):
        self.timeout=timeout;self.seen={};self.active=False;self.latched=False
    def touch(self,key,now):self.seen[key]=now
    def ready(self,now):
        ready=all(k in self.seen and 0<=now-self.seen[k]<self.timeout for k in ['wheel','arm','top'])
        if self.active and not ready:self.latched=True
        if ready:self.active=True
        return ready and not self.latched


class FusedGate(Node):
    def __init__(self):
        super().__init__('fused_odom_gate');self.health=Freshness();self.latest=None
        self.pub=self.create_publisher(Odometry,'/odometry/filtered',20)
        self.diag=self.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.tf=TransformBroadcaster(self)
        self.subs=[self.create_subscription(Odometry,'/wheel/odometry',lambda m:self.touch('wheel',m),20),
                   self.create_subscription(Imu,'/fusion/imu/arm_yaw_rate',lambda m:self.touch('arm',m),100),
                   self.create_subscription(Imu,'/fusion/imu/top_yaw_rate',lambda m:self.touch('top',m),100),
                   self.create_subscription(Odometry,'/fusion/odometry_internal',self.receive,20)]
        self.timer=self.create_timer(.02,self.check)
        self.dtimer=self.create_timer(1.,self.diagnostic)

    def touch(self,key,m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if -.05<=age<.25:self.health.touch(key,time.monotonic())

    def check(self):self.health.ready(time.monotonic())

    def receive(self,m):
        self.latest=time.monotonic()
        if not self.health.ready(self.latest):return
        p=m.pose.pose.position;q=m.pose.pose.orientation;t=m.twist.twist
        if m.header.frame_id!='odom' or m.child_frame_id!='base_link':return
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,t.linear.x,t.linear.y,t.angular.z,*m.pose.covariance,*m.twist.covariance]
        if not np.isfinite(values).all():return
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if not -.05<=self.get_clock().now().nanoseconds*1e-9-stamp<.25:return
        self.pub.publish(m)
        tf=TransformStamped();tf.header=m.header;tf.child_frame_id=m.child_frame_id
        tf.transform.translation.x=p.x;tf.transform.translation.y=p.y;tf.transform.translation.z=p.z;tf.transform.rotation=q
        self.tf.sendTransform(tf)

    def diagnostic(self):
        now=time.monotonic();ready=self.health.ready(now) and self.latest is not None and now-self.latest<.25
        d=DiagnosticStatus();d.name='fusion/final_odometry';d.hardware_id='wheel+dual_livox_imu'
        d.level=DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
        d.message='VALID_FUSED_ODOMETRY' if ready else ('FAULT_LATCHED_RESTART_REQUIRED' if self.health.latched else 'WAITING_FOR_WHEEL_AND_BOTH_IMUS')
        d.values=[KeyValue(key=k,value=str(now-v)) for k,v in self.health.seen.items()]
        a=DiagnosticArray();a.header.stamp=self.get_clock().now().to_msg();a.status=[d];self.diag.publish(a)


if __name__=='__main__':
    rclpy.init();n=FusedGate()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
