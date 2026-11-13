#!/usr/bin/env python3
"""Causal velocity smoothing; keeps poses and conservative input covariance."""
import argparse
import copy
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def lowpass(previous, current, dt, tau):
    return previous + (-math.expm1(-dt/tau))*(current-previous)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--sensor', choices=['arm', 'top'], required=True)
    p.add_argument('--tau', type=float, default=.2)
    a = p.parse_args()
    assert a.tau > 0
    rclpy.init()
    node = Node(a.sensor+'_twist_smoother')
    pub = node.create_publisher(Odometry, f'/dual/{a.sensor}/odometry_smoothed', 10)
    state = {'stamp': None, 'velocity': None}
    def receive(msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        tw = msg.twist.twist
        velocity = np.array([tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z])
        if not np.isfinite(velocity).all():
            return
        if state['stamp'] is not None:
            dt = stamp-state['stamp']
            if not 0 < dt <= 1:
                state.update(stamp=None, velocity=None)
                return
            velocity = lowpass(state['velocity'], velocity, dt, a.tau)
        state.update(stamp=stamp, velocity=velocity)
        out = copy.deepcopy(msg)
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, velocity[:3])
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, velocity[3:])
        pub.publish(out)
    sub = node.create_subscription(Odometry, f'/dual/{a.sensor}/odometry', receive, 50)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
