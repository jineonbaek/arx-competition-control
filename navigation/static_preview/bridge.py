"""Sensor-centred gravity frame and finite-return scan; no actuator interface."""
import os,json,time,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener,TransformBroadcaster
from geometry_msgs.msg import TransformStamped,PolygonStamped,Point32
from sensor_msgs.msg import PointCloud2,LaserScan
from sensor_msgs_py.point_cloud2 import read_points_numpy,create_cloud_xyz32
from core import sanitize_ranges,split_ground
class Bridge(Node):
 def __init__(self):
  super().__init__('preview_level_bridge');self.buf=Buffer();self.listener=TransformListener(self.buf,self);self.br=TransformBroadcaster(self)
  self.pub=self.create_publisher(PointCloud2,'/preview/level_cloud',10);self.scanpub=self.create_publisher(LaserScan,'/preview/scan',10)
  self.sub=self.create_subscription(PointCloud2,'/single/cloud_body',self.cloud,qos_profile_sensor_data)
  self.ss=self.create_subscription(LaserScan,'/preview/scan_unchecked',self.scan,qos_profile_sensor_data)
  self.footpub=self.create_publisher(PolygonStamped,"/preview/footprint",1);self.foottimer=self.create_timer(.5,self.footprint)
  self.pairs={};self.safeobs=self.create_publisher(PointCloud2,'/preview/obstacles',10);self.safeground=self.create_publisher(PointCloud2,'/preview/ground',10)
  self.gs=self.create_subscription(PointCloud2,'/preview/ground_raw',lambda m:self.segment(m,'ground'),qos_profile_sensor_data)
  self.os=self.create_subscription(PointCloud2,'/preview/obstacle_raw',lambda m:self.segment(m,'obstacle'),qos_profile_sensor_data)
  self.count=0;self.drops=0;self.last=0;self.pending=None;self.timer=self.create_timer(.02,self.tick)
 def footprint(self):
  pts=json.loads((Path(__file__).resolve().parent/"footprint.json").read_text())["polygon_m"]
  p=PolygonStamped();p.header.frame_id="base_link";p.header.stamp=self.get_clock().now().to_msg();p.polygon.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in pts];self.footpub.publish(p)
 def cloud(self,m):self.pending=m
 def tick(self):
  if self.pending is None:return
  m=self.pending
  age=(self.get_clock().now().nanoseconds-Time.from_msg(m.header.stamp).nanoseconds)/1e9
  if age>1 or age<-.1:self.pending=None;self.drops+=1;return
  stamp=Time.from_msg(m.header.stamp)
  try:t=self.buf.lookup_transform('odom','bottom_livox',stamp);src=self.buf.lookup_transform('bottom_livox',m.header.frame_id,stamp)
  except Exception:return
  self.pending=None
  q=t.transform.rotation;s=src.transform.rotation;v=src.transform.translation
  pts=read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
  # First IMU cloud -> actual optical origin; then gravity align about that origin.
  pts=(pts@Rotation.from_quat([s.x,s.y,s.z,s.w]).as_matrix().T+np.array([v.x,v.y,v.z]))@Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix().T
  tf=TransformStamped();tf.header=m.header;tf.header.frame_id='odom';tf.child_frame_id='preview_lidar_level';tf.transform.translation=t.transform.translation;tf.transform.rotation.w=1.;self.br.sendTransform(tf)
  h=m.header;h.frame_id='preview_lidar_level';self.pub.publish(create_cloud_xyz32(h,pts.astype(np.float32)));self.count+=1
  if self.count%50==0:Path(os.environ['ARX_PREVIEW_RUN'],'bridge-health.json').write_text(json.dumps({'frames':self.count,'stale_dropped':self.drops,'stamp':stamp.nanoseconds,'optical_height_m':t.transform.translation.z,'commands_sent':0}))
 def segment(self,m,key):
  stamp=(m.header.stamp.sec,m.header.stamp.nanosec);pair=self.pairs.setdefault(stamp,{})
  pair[key]=m
  if len(self.pairs)>20:self.pairs.pop(next(iter(self.pairs)))
  if len(pair)!=2:return
  self.pairs.pop(stamp,None)
  ground=read_points_numpy(pair['ground'],field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
  obstacles=read_points_numpy(pair['obstacle'],field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
  good,uncertain=split_ground(ground)
  out=np.vstack([obstacles,np.array(uncertain).reshape(-1,3)])
  self.safeobs.publish(create_cloud_xyz32(m.header,out.astype(np.float32)))
  self.safeground.publish(create_cloud_xyz32(m.header,np.array(good,np.float32).reshape(-1,3)))
 def scan(self,m):
  m.ranges=sanitize_ranges(m.ranges,m.range_min,m.range_max);self.scanpub.publish(m)
if __name__=='__main__':
 rclpy.init();n=Bridge()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
