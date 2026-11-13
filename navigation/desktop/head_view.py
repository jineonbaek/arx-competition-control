"""Head-only diagnostic display in the lidar frame; independent of bottom odometry."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


class HeadView(Node):
    def __init__(self):
        super().__init__('head_only_raw_view')
        self.pub=self.create_publisher(PointCloud2,'/diagnostics/head_points',qos_profile_sensor_data)
        self.create_subscription(CustomMsg,'/unused_head/lidar',self.frame,qos_profile_sensor_data)

    def frame(self,m):
        if self.pub.get_subscription_count():
            self.pub.publish(create_cloud_xyz32(Header(stamp=m.header.stamp,frame_id='head_native'),
                                                [(p.x,p.y,p.z) for p in m.points]))

if __name__=='__main__':
    rclpy.init();node=HeadView()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
