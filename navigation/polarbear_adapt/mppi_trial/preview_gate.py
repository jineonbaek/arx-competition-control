"""Bound MPPI output and report stale inputs on a preview-only topic."""
import math
import time
import rclpy
from geometry_msgs.msg import TwistStamped
from sdk_preview import convert
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PreviewGate(Node):
    def __init__(self):
        super().__init__('arx_mppi_preview_gate')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.inputs = {}
        self.previous_reason = None
        self.output = self.create_publisher(TwistStamped, '/arx_mppi_trial/preview_velocity', 1)
        self.sdk_output = self.create_publisher(TwistStamped, '/arx_mppi_trial/sdk_preview', 1)
        self.status = self.create_publisher(String, '/arx_mppi_trial/preview_status', 1)
        for name, cls, topic, qos in (
            ('velocity', TwistStamped, '/arx_mppi_trial/raw_velocity', 10),
            ('odometry', Odometry, '/localization/odometry', qos_profile_sensor_data),
            ('front', LaserScan, '/pb/scan', qos_profile_sensor_data),
            ('rear', LaserScan, '/pb/head/direct_scan', qos_profile_sensor_data),
            ('costmap', OccupancyGrid, '/arx_mppi_trial/local_costmap/costmap',
             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ):
            self.create_subscription(cls, topic, lambda msg,k=name:self.receive(k,msg), qos)
        self.create_timer(.05, self.update)

    def receive(self, name, msg):
        ns = Time.from_msg(msg.header.stamp).nanoseconds
        if name not in self.inputs or ns > self.inputs[name][0]:
            self.inputs[name] = (ns, time.monotonic(), msg)

    def blocked(self, now):
        wall = time.monotonic()
        for name, age, gap in (('velocity',.20,.15), ('odometry',.45,.25),
                               ('front',.65,.25), ('rear',.65,.25), ('costmap',.50,.35)):
            ns, arrived, _ = self.inputs.get(name, (0,0,None))
            if not 0 <= (now-ns)/1e9 <= age or wall-arrived > gap:
                return name + '_stale'
        try:
            pose = self.buf.lookup_transform('odom','base_link',Time())
        except TransformException:
            return 'tf_missing'
        if not 0 <= (now-Time.from_msg(pose.header.stamp).nanoseconds)/1e9 <= .45:
            return 'tf_stale'
        return None

    def update(self):
        stamp = self.get_clock().now()
        msg = TwistStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'base_link'
        reason = self.blocked(stamp.nanoseconds)
        if reason is None:
            twist = self.inputs['velocity'][2].twist
            x,y,w = twist.linear.x,twist.linear.y,twist.angular.z
            if all(math.isfinite(v) for v in (x,y,w)):
                scale = min(1., .10/max(math.hypot(x,y),1e-9))
                msg.twist.linear.x = x*scale
                msg.twist.linear.y = y*scale
                msg.twist.angular.z = max(-.18,min(w,.18))
            else:
                reason = 'velocity_invalid'
        self.output.publish(msg)
        sdk = TwistStamped()
        sdk.header = msg.header
        v = msg.twist
        sdk.twist.linear.x, sdk.twist.linear.y, sdk.twist.angular.z = convert(
            v.linear.x, v.linear.y, v.angular.z)
        self.sdk_output.publish(sdk)
        reason = reason or 'preview_ready'
        if reason != self.previous_reason:
            self.get_logger().info(reason)
            self.previous_reason = reason
        self.status.publish(String(data=reason))


if __name__ == '__main__':
    rclpy.init()
    node = PreviewGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
