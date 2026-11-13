"""Reconstruct only chassis TF from recorded odometry. Isolated replay only."""
import os
if os.getenv('ROS_DOMAIN_ID')!='199':raise RuntimeError('Replay TF requires domain199')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
class ReplayTF(Node):
 def __init__(self):
  super().__init__('pb_replay_tf');self.br=TransformBroadcaster(self);self.st=StaticTransformBroadcaster(self);rows=[]
  for parent,child,v in [('base_link','bottom_imu',[.196330,.023287,.185880]),('bottom_imu','bottom_livox',[-.011,-.02329,.04412])]:
   t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=v;t.transform.rotation.w=1.;rows.append(t)
  self.st.sendTransform(rows);self.sub=self.create_subscription(Odometry,'/localization/odometry',self.receive,100)
 def receive(self,m):
  t=TransformStamped();t.header=m.header;t.child_frame_id='base_link';p=m.pose.pose.position;t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.translation.z=p.z;t.transform.rotation=m.pose.pose.orientation;self.br.sendTransform(t)
rclpy.init();n=ReplayTF()
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.try_shutdown()
