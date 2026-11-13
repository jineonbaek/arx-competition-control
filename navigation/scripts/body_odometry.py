#!/usr/bin/env python3
"""Rigid-body lever-arm correction. Fails closed without measured calibration.

Outputs are separate estimates, not fusion. A shared world alignment is required
before fusing poses. Twist includes rotational velocity at the sensor lever arm.
"""
from pathlib import Path
import argparse
import copy
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def skew(t):
    x, y, z = t
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def transform_state(position, quaternion, twist, translation, rotation):
    Rwi = Rotation.from_quat(quaternion).as_matrix()
    Rbi = Rotation.from_euler('xyz', rotation).as_matrix()
    t = np.asarray(translation)
    Rwb = Rwi @ Rbi.T
    p = np.asarray(position) - Rwb @ t
    J = np.zeros((6, 6))
    J[:3, :3] = Rbi
    J[:3, 3:] = skew(t) @ Rbi
    J[3:, 3:] = Rbi
    return p, Rotation.from_matrix(Rwb).as_quat(), J @ twist, J, Rwb


def validate(cfg):
    if cfg.get('calibration_validated') is not True:
        raise ValueError('Measured calibration_validated:true required; no fake base_link output')
    if cfg.get('primary') not in ('arm', 'top'):
        raise ValueError('Select primary arm or top')
    if cfg.get('arm_lift_locked') is not True:
        raise ValueError('Rigid arm extrinsic requires locked lift; dynamic lift TF not implemented')
    for name in ('arm', 'top'):
        for field in ('xyz_m', 'rpy_rad'):
            v = cfg[name][field]
            if not isinstance(v, list) or len(v) != 3 or not np.isfinite(v).all():
                raise ValueError(f'{name}.{field} must contain three measured finite values')


class BodyOdometry(Node):
    def __init__(self, cfg):
        super().__init__('body_lidar_odometry')
        self.cfg = cfg
        self.tf = TransformBroadcaster(self)
        self.pubs = {n: self.create_publisher(Odometry, f'/dual/{n}/body_odometry', 10) for n in ('arm', 'top')}
        self.primary = self.create_publisher(Odometry, '/localization/odometry', 10)
        self.last = {}
        self.subs = [self.create_subscription(Odometry, f'/dual/{n}/odometry_smoothed', lambda m, n=n: self.receive(n, m), 10) for n in ('arm', 'top')]

    def receive(self, name, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if name in self.last and not 0 < stamp - self.last[name] < 1.0:
            self.get_logger().error(f'{name}: discontinuity, reinitialization required')
            return
        self.last[name] = stamp
        po, tw = msg.pose.pose, msg.twist.twist
        p, q, vel, J, Rwb = transform_state(
            [po.position.x, po.position.y, po.position.z],
            [po.orientation.x, po.orientation.y, po.orientation.z, po.orientation.w],
            np.array([tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z]),
            self.cfg[name]['xyz_m'], self.cfg[name]['rpy_rad'])
        out = copy.deepcopy(msg)
        out.child_frame_id = f'{name}_base_estimate'
        for obj, vals in [(out.pose.pose.position, p), (out.twist.twist.linear, vel[:3]), (out.twist.twist.angular, vel[3:])]:
            obj.x, obj.y, obj.z = map(float, vals)
        out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = map(float, q)
        out.twist.covariance = (J @ np.asarray(msg.twist.covariance).reshape(6, 6) @ J.T).ravel().tolist()
        Jp = np.eye(6)
        Jp[:3, 3:] = skew(Rwb @ np.asarray(self.cfg[name]['xyz_m']))
        out.pose.covariance = (Jp @ np.asarray(msg.pose.covariance).reshape(6, 6) @ Jp.T).ravel().tolist()
        self.pubs[name].publish(out)
        if name == self.cfg['primary']:
            out.child_frame_id = 'base_link'
            self.primary.publish(out)
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = out.child_frame_id
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
            t.transform.rotation = out.pose.pose.orientation
            self.tf.sendTransform(t)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', type=Path, required=True)
    args = parser.parse_args()
    cfg = yaml.safe_load(args.calibration.read_text())
    validate(cfg)
    rclpy.init()
    node = BodyOdometry(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
