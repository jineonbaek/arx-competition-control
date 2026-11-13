#!/usr/bin/env python3
"""Stationary calibrated gyro projection for planar body yaw rate.

Output is a virtual body-Z-only IMU measurement. No invented absolute heading,
linear acceleration, full mounting transform, or translational lever arm.
"""
from pathlib import Path
import argparse
import copy
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from odometry_math import project_gyro_z


class ImuYaw(Node):
    def __init__(self, calibration, name):
        super().__init__(name+'_planar_imu_adapter')
        self.cal=calibration['sensors'][name]
        if not calibration.get('operator_confirmed_static') or not self.cal.get('valid'):
            raise ValueError('Operator-confirmed valid static calibration required')
        boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        if calibration.get('boot_id') != boot:
            raise ValueError('Recalibrate gyro bias after NUC/sensor power cycle')
        self.name_=name
        self.bias=np.array(self.cal['gyro_bias_native_rad_s'])
        self.up=np.array(self.cal['up_axis_native'])
        self.cov=np.array(self.cal['gyro_covariance_native'])
        self.last=None
        self.fault=False
        self.pub=self.create_publisher(Imu,f'/fusion/imu/{name}_yaw_rate',100)
        self.si=self.create_publisher(Imu,f'/fusion/imu/{name}_raw_si',100)
        self.sub=self.create_subscription(Imu,f'/dual/{name}/imu',self.receive,500)

    def receive(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.last is not None and not 0<stamp-self.last<.25:
            self.fault=True
            self.get_logger().error('IMU timestamp discontinuity: adapter latched; reinitialize')
        self.last=stamp
        if self.fault:return
        w=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        acc=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        if not np.isfinite([*w,*acc]).all():return
        raw=copy.deepcopy(msg)
        raw.header.frame_id=self.name_+'_lio_imu'
        raw.orientation.x=raw.orientation.y=raw.orientation.z=0.0;raw.orientation.w=1.0
        raw.orientation_covariance=[-1.0,0.,0.,0.,0.,0.,0.,0.,0.]
        raw.angular_velocity_covariance=(self.cov*2.0).ravel().tolist()
        raw.linear_acceleration.x,raw.linear_acceleration.y,raw.linear_acceleration.z=map(float,acc*9.80665)
        raw.linear_acceleration_covariance=np.diag((np.array(self.cal['acceleration_std_g'])*9.80665)**2*2).ravel().tolist()
        self.si.publish(raw)
        wz,var=project_gyro_z(w,self.bias,self.up,self.cov)
        if abs(wz)>10:return
        out=Imu();out.header=copy.deepcopy(msg.header);out.header.frame_id='base_link'
        out.orientation.w=1.0
        out.orientation_covariance=[-1.0,0.,0.,0.,0.,0.,0.,0.,0.]
        out.angular_velocity.z=wz
        C=np.diag([1e6,1e6,var+.005**2]);out.angular_velocity_covariance=C.ravel().tolist()
        out.linear_acceleration_covariance=[-1.0,0.,0.,0.,0.,0.,0.,0.,0.]
        self.pub.publish(out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--calibration',type=Path,required=True);p.add_argument('--sensor',choices=['arm','top'],required=True);a=p.parse_args()
    rclpy.init();n=ImuYaw(json.loads(a.calibration.read_text()),a.sensor)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
