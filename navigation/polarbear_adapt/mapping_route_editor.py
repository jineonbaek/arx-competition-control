"""Pure mapping route editor: no chassis transport, goal execution or task worker."""
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from chassis_pose import valid_pose,planar
from chassis_pose_policy import MOTION_MAX_AGE_NS

class EditorHub:
    def set_route(self,*args,**kwargs):pass
    def state(self):return dict(task=None)
    def blocking(self):return False
    def available_points(self):return []
class EditorRuntime:
    def __init__(self):self.hub=EditorHub()
    def close(self):pass
    def cancel(self):pass

class MappingRouteEditor(Node):
    def __init__(self):
        super().__init__('pb_mapping_route_editor')
        from waypoint_controls import Waypoints
        self.target=None;self.active=False;self.ready_file=None;self.snapshot=None
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseArray,'/pb/chassis_pose',lambda m:setattr(self,'snapshot',m),qos)
        self.state=self.create_publisher(String,'/pb/drive_status',qos)
        self.create_timer(1.,lambda:self.state.publish(String(data='mapping_editor_idle')))
        self.waypoints=Waypoints(self,mapping_only=True)
    def special_poses(self):
        m=self.snapshot
        if m is None or m.header.frame_id!='map' or len(m.poses)!=2 or not all(valid_pose(p) for p in m.poses):raise ValueError('当前位置无效')
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if not 0<=age<=MOTION_MAX_AGE_NS/1e9:raise ValueError('当前位置过期')
        return np.asarray(planar(m.poses[0])),np.asarray(planar(m.poses[1])),stamp
    def destroy_node(self):
        self.waypoints.close();return super().destroy_node()

def main():
    if os.getenv('ROS_DOMAIN_ID') not in ('192','197'):raise SystemExit('Explicit navigation or isolated domain required')
    rclpy.init();n=MappingRouteEditor()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
