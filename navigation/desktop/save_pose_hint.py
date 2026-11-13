"""Record a recent map->base_link transform as a localization seed for this map."""
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    rclpy.init()
    node = rclpy.create_node("save_navigation_pose_hint")
    buf = Buffer()
    listener = TransformListener(buf, node)
    try:
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tf = buf.lookup_transform("map", "base_link", Time())
            except TransformException:
                continue
            stamp = Time.from_msg(tf.header.stamp)
            age = (node.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
            if not -0.1 <= age <= 0.5:
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
                continue
            with Path(sys.argv[1]).open("x") as out:
                json.dump(dict(frame="map", child_frame="base_link", x_m=p.x, y_m=p.y,
                               yaw_rad=yaw, stamp_ns=stamp.nanoseconds), out, indent=2)
            return 0
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
