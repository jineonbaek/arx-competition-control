"""Check live sensor messages and timestamped TF in the desktop ROS domain."""
import argparse
from collections import deque
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=("raw", "lio", "mapping", "navigation"))
    parser.add_argument("--head", action="store_true")
    parser.add_argument("--dual-origin", action="store_true")
    parser.add_argument("--reuse", action="store_true")
    parser.add_argument("--timeout", type=float, default=7)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("arx_desktop_chain_health")
    tf = Buffer()
    listener = TransformListener(tf, node)
    samples = {}
    required = {}

    def receive(topic, msg):
        ns = Time.from_msg(msg.header.stamp).nanoseconds
        previous, count, _ = samples.get(topic, (0, 0, 0))
        if ns > previous:
            samples[topic] = ns, count + 1, time.monotonic()
            if topic == "/pb/mapping_scan":
                scan_frames.append((ns, msg.header.frame_id))

    def watch(topic, cls, count=3, qos=qos_profile_sensor_data, age=1.):
        required[topic] = (count, age)
        node.create_subscription(cls, topic, lambda msg: receive(topic, msg), qos)

    ready_at = None

    def dual_ready(msg):
        nonlocal ready_at
        ready_at = time.monotonic() if msg.data else None

    if args.dual_origin:
        node.create_subscription(Bool, "/pb/dual_origin/ready", dual_ready, 1)

    scan_frames = deque(maxlen=12)
    watch("/single/lidar_raw", CustomMsg)
    watch("/single/imu", Imu)
    if args.head:
        watch("/unused_head/lidar", CustomMsg)
    if args.stage != "raw":
        watch("/localization/odometry", Odometry)
    if args.stage in ("mapping", "navigation"):
        watch("/pb/mapping_scan", LaserScan)
        watch("/pb/map", OccupancyGrid, 1, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), 12.)
    if args.stage == "navigation":
        watch("/amcl_pose", PoseWithCovarianceStamped, 1,
              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
              float("inf") if args.reuse else 45.)

    def missing():
        now = node.get_clock().now().nanoseconds
        wall = time.monotonic()
        not_ready = [
            topic for topic, (minimum, age) in required.items()
            if (count := samples.get(topic, (0, 0, 0)))[1] < minimum
            or wall - count[2] > age
            or not -0.3 <= (now - count[0]) / 1e9 <= age
        ]

        if args.dual_origin and (ready_at is None or wall - ready_at > .8):
            not_ready.append("双光心地图（前后扫描未就绪）")
        return not_ready

    end = time.monotonic() + args.timeout
    failure = []
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.06)
            failure = missing()
            if failure:
                continue
            try:
                if args.stage in ("mapping", "navigation"):
                    for ns, frame in reversed(scan_frames):
                        try:
                            tf.lookup_transform("odom", frame, Time(nanoseconds=ns))
                            break
                        except TransformException:
                            continue
                    else:
                        failure = ["odom → scan (扫描时间)"]
                        continue
                    pose = tf.lookup_transform("map", "base_link", Time())
                    age = (node.get_clock().now().nanoseconds - Time.from_msg(pose.header.stamp).nanoseconds) / 1e9
                    if not -0.5 <= age <= 0.7:
                        failure = ["map → base_link (位姿已过期)"]
                        continue
                return 0
            except TransformException:
                failure = ["map → base_link"]
        print("未就绪：" + "、".join(failure), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
