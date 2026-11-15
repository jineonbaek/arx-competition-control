"""Exercise the real ROS gate before a controller has emitted any velocity."""
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from preview_gate import PreviewGate


class GateReadiness(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = PreviewGate()
        self.stamp = self.gate.get_clock().now()
        for key, kind in [('front', LaserScan), ('rear', LaserScan),
                          ('odometry', Odometry), ('costmap', OccupancyGrid)]:
            msg = kind()
            msg.header.stamp = self.stamp.to_msg()
            self.gate.receive(key, msg)
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.header.stamp = self.stamp.to_msg()
        transform.transform.rotation.w = 1.
        self.gate.buf.set_transform(transform, 'test')

    def tearDown(self):
        self.gate.destroy_node()

    def test_sensor_readiness_does_not_require_a_command(self):
        self.assertIsNone(self.gate.blocked(self.stamp.nanoseconds, require_velocity=False))
        self.assertEqual(self.gate.blocked(self.stamp.nanoseconds), 'velocity_stale')

    def test_startup_still_rejects_old_scan_and_reception_gap(self):
        self.assertEqual(self.gate.blocked(self.stamp.nanoseconds + 700_000_000,
                                          require_velocity=False), 'odometry_stale')
        ns, _, msg = self.gate.inputs['rear']
        self.gate.inputs['rear'] = (ns, time.monotonic()-1., msg)
        self.assertEqual(self.gate.blocked(self.stamp.nanoseconds, require_velocity=False), 'rear_stale')


if __name__ == '__main__':
    unittest.main()
