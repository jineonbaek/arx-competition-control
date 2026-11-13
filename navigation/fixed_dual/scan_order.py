"""Publish the two mapping scans in acquisition timestamp order."""

import heapq
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Order(Node):
    def __init__(self):
        super().__init__("fixed_dual_scan_order")
        hold_ms = self.declare_parameter("reorder_hold_ms", 150).value
        if not 0 <= hold_ms <= 150:
            raise ValueError("reorder_hold_ms must be between 0 and 150")
        self.hold_ns = hold_ms * 1_000_000
        self.queue = []
        self.sequence = 0
        self.last_stamp = 0
        self.drops = {"bottom": 0, "head": 0}
        self.max_wait_ms = 0.0
        self.next_report = time.monotonic() + 10.0
        self.publisher = self.create_publisher(LaserScan, "/pb/mapping_scan", 10)
        self.scan_subscriptions = [
            self.create_subscription(
                LaserScan,
                f"/fixed_dual/{source}/mapping_scan",
                lambda scan, source=source: self.receive(scan, source),
                qos_profile_sensor_data,
            )
            for source in self.drops
        ]
        self.create_timer(0.02, self.tick)

    def receive(self, scan, source):
        stamp = scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec
        self.sequence += 1
        heapq.heappush(self.queue, (stamp, self.sequence, time.monotonic_ns(), source, scan))
        if len(self.queue) > 40:
            _, _, _, dropped_source, _ = heapq.heappop(self.queue)
            self.drops[dropped_source] += 1

    def tick(self):
        now = time.monotonic_ns()
        while self.queue and now - self.queue[0][2] >= self.hold_ns:
            stamp, _, arrived, source, scan = heapq.heappop(self.queue)
            if stamp < self.last_stamp:
                self.drops[source] += 1
                continue
            self.publisher.publish(scan)
            self.last_stamp = stamp
            self.max_wait_ms = max(self.max_wait_ms, (now - arrived) / 1e6)
        if now / 1e9 >= self.next_report:
            self.get_logger().info(
                f"Scan order drops: {self.drops}; max queue wait: {self.max_wait_ms:.1f} ms"
            )
            self.next_report = now / 1e9 + 10.0
            self.max_wait_ms = 0.0


def main():
    rclpy.init()
    node = Order()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
