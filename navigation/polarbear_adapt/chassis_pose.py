"""Single source-stamped pose/validity snapshot shared by all body-free consumers.

/pb/chassis_pose PoseArray: frame map, [map<-base, odom<-base]. Empty = invalid.
No extrapolation, command publishers, free-space caching or independent TF tests
in consumers. A dead authority/source expires at the 1s motion-source deadline.
"""
import math
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseArray,PoseWithCovarianceStamped,Pose
from nav_msgs.msg import Odometry
from tf2_ros import Buffer,TransformListener,TransformException
from scipy.spatial.transform import Rotation
import numpy as np
import os
from chassis_pose_policy import PosePolicy

TOPIC='/pb/chassis_pose'

def planar(p):
    q=p.orientation
    return (p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

def valid_pose(p):
    a=p.position;q=p.orientation
    return (all(math.isfinite(v) for v in (a.x,a.y,a.z,q.x,q.y,q.z,q.w)) and
            abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<.02)

def transform_pose(transform,pose):
    t=transform.translation;q=transform.rotation
    r=Rotation.from_quat([q.x,q.y,q.z,q.w]);a=pose.position;b=pose.orientation
    v=r.apply([a.x,a.y,a.z])+[t.x,t.y,t.z]
    result=Pose();result.position.x,result.position.y,result.position.z=map(float,v)
    quat=(r*Rotation.from_quat([b.x,b.y,b.z,b.w])).as_quat()
    result.orientation.x,result.orientation.y,result.orientation.z,result.orientation.w=map(float,quat)
    return result

class PoseAuthority(Node):
    def __init__(self):
        super().__init__('pb_chassis_pose')
        self.policy=PosePolicy();self.latest=None;self.awaiting=None;self.after=None
        self.saved=bool(os.getenv('ARX_SAVED_MAP'))
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(PoseArray,TOPIC,qos)
        self.create_subscription(Odometry,'/localization/odometry',self.receive,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.relocalize,10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.localized,10)
        self.create_timer(.02,self.publish)
    def receive(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link' or not valid_pose(msg.pose.pose):
            self.latest=None;self.policy.reject();return
        self.latest=msg
    def relocalize(self,msg):
        if msg.header.frame_id!='map':return
        self.after=self.get_clock().now().nanoseconds
        self.awaiting=self.after if self.saved else None
        self.policy.reset();self.publish_invalid()
    def localized(self,msg):
        if (self.awaiting is not None and msg.header.frame_id=='map' and
            Time.from_msg(msg.header.stamp).nanoseconds>=self.awaiting):self.awaiting=None
    def publish_invalid(self):
        m=PoseArray();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();self.pub.publish(m)
    def publish(self):
        msg=self.latest
        if msg is None or self.awaiting is not None:
            self.policy.reject();self.publish_invalid();return
        stamp=Time.from_msg(msg.header.stamp)
        if self.after is not None:
            if stamp.nanoseconds<=self.after:self.publish_invalid();return
            self.after=None
        try:
            tf=self.buffer.lookup_transform('map','odom',stamp).transform
            # Validate alignment as a pose before scipy normalizes its quaternion.
            alignment=Pose();alignment.position.x=tf.translation.x;alignment.position.y=tf.translation.y;alignment.position.z=tf.translation.z;alignment.orientation=tf.rotation
            if not valid_pose(alignment):raise ValueError('invalid alignment')
            map_pose=transform_pose(tf,msg.pose.pose)
        except (TransformException,ValueError):
            self.policy.reject();self.publish_invalid();return
        valid=self.policy.accept(stamp.nanoseconds,self.get_clock().now().nanoseconds,planar(alignment))
        m=PoseArray();m.header.frame_id='map';m.header.stamp=msg.header.stamp
        if valid:m.poses=[map_pose,copy.deepcopy(msg.pose.pose)]
        self.pub.publish(m)

def main():
    rclpy.init();n=PoseAuthority()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
