#!/usr/bin/env python3
"""Perception-only conversion: LIO keeps the original per-point CustomMsg."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud


def to_cloud(msg):
    fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    points = np.asarray([(p.x, p.y, p.z, float(p.reflectivity)) for p in msg.points],
                        dtype=np.float32).reshape((-1, 4))
    return create_cloud(msg.header, fields, points)


class Converter(Node):
    def __init__(self):
        super().__init__('livox_custom_to_cloud')
        self.publisher = self.create_publisher(
            PointCloud2, '/livox/lidar/pointcloud', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            CustomMsg, '/livox/lidar', lambda msg: self.publisher.publish(to_cloud(msg)),
            qos_profile_sensor_data)


def main():
    rclpy.init()
    node = Converter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
