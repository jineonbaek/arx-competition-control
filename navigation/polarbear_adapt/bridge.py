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
from sensor_msgs.msg import PointCloud2,PointField,LaserScan
from sensor_msgs_py.point_cloud2 import read_points_numpy,create_cloud_xyz32
from shared_obstacles import packet
from global_mark import GlobalMarkFilter
import os
from chassis_filter import ChassisFilter,bottom_left_self_returns,bottom_right_weak_returns
from shared_voxels import without_self_shadow
from core import split_ground
from blind_core import filter_scan,apply_occlusion
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rear_sector import rear_points,mask_scan
class Bridge(Node):
 def __init__(self):
  self.source=os.getenv('ARX_OBSERVATION_SENSOR','bottom');self.rear=self.source=='head';self.optical='rear_livox' if self.rear else 'bottom_livox';self.level='pb_rear_lidar_level' if self.rear else 'pb_lidar_level'
  super().__init__('pb_rear_level_bridge' if self.rear else 'pb_level_bridge');self.buf=Buffer();self.listener=TransformListener(self.buf,self);self.br=TransformBroadcaster(self)
  model=Path(__file__).resolve().parents[1]/"models/lift2s_bottom_upright_20261105/body_voxels.npz"
  sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'fixed_dual'))
  from geometry import enabled,configuration
  self.dual=enabled(Path(__file__).resolve().parents[1])
  if self.dual:model=Path(__file__).resolve().parents[1]/configuration(Path(__file__).resolve().parents[1])['mesh']
  self.mesh=None
  if self.dual:
   sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'dual_flex'))
   from mesh_filter import MeshFilter
   self.mesh=MeshFilter.load(model)
  self.clearance=None
  if os.getenv("ARX_REPLAY")=="1" and os.getenv("ARX_EXPERIMENTAL_CLEAR")=="1":
   from clearance_bridge import ClearanceBridge
   self.clearance=ClearanceBridge(self)
  self.chassis=ChassisFilter(model);self.chassis_dropped=0;self.right_weak_dropped=0;self.left_self_dropped=0
  self.pub=self.create_publisher(PointCloud2,'/pb/level_cloud',10);self.scanpub=self.create_publisher(LaserScan,'/pb/scan',10)
  self.sub=self.create_subscription(PointCloud2,f'/fixed_dual/{self.source}/cloud_body' if self.dual else '/single/cloud_body',self.cloud,qos_profile_sensor_data)
  self.ss=self.create_subscription(LaserScan,'/pb/scan_unchecked',self.scan,qos_profile_sensor_data)
  self.footpub=self.create_publisher(PolygonStamped,"/pb/footprint",1);self.foottimer=self.create_timer(.5,self.footprint)
  self.shared_pub=self.create_publisher(PointCloud2,"/pb/bottom_observations",2);self.rays={}
  self.global_filter=GlobalMarkFilter()
  self.global_pub=self.create_publisher(PointCloud2,"/pb/global_mark",10)
  self.pairs={};self.safeobs=self.create_publisher(PointCloud2,'/pb/obstacles',10);self.safeground=self.create_publisher(PointCloud2,'/pb/ground',10)
  self.gs=self.create_subscription(PointCloud2,'/pb/ground_raw',lambda m:self.segment(m,'ground'),qos_profile_sensor_data)
  self.os=self.create_subscription(PointCloud2,'/pb/obstacle_raw',lambda m:self.segment(m,'obstacle'),qos_profile_sensor_data)
  self.blind=float(os.environ.get('ARX_BLIND','0.35'))
  self.mapscanpub=self.create_publisher(LaserScan,'/pb/mapping_scan',10)
  self.ms=self.create_subscription(LaserScan,'/pb/mapping_scan_unchecked',self.mapping_scan,qos_profile_sensor_data)
  self.near_masks={};self.count=0;self.drops=0;self.last=0;self.pending=None;self.timer=self.create_timer(.02,self.tick)
 def footprint(self):
  if self.rear:return
  pts=json.loads((Path(__file__).resolve().parent/"footprint.json").read_text())["polygon_m"]
  p=PolygonStamped();p.header.frame_id="base_link";p.header.stamp=self.get_clock().now().to_msg();p.polygon.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in pts];self.footpub.publish(p)
 def cloud(self,m):self.pending=m
 def tick(self):
  if self.pending is None:return
  m=self.pending
  age=(self.get_clock().now().nanoseconds-Time.from_msg(m.header.stamp).nanoseconds)/1e9
  if age>1 or age<-.1:self.pending=None;self.drops+=1;return
  stamp=Time.from_msg(m.header.stamp)
  try:t=self.buf.lookup_transform('odom',self.optical,stamp);src=self.buf.lookup_transform(self.optical,m.header.frame_id,stamp);body=self.buf.lookup_transform('base_link',self.optical,stamp)
  except Exception:return
  self.pending=None
  q=t.transform.rotation;s=src.transform.rotation;v=src.transform.translation
  with_strength=any(f.name=='intensity' and f.datatype==PointField.FLOAT32 and f.count==1 for f in m.fields)
  fields=('x','y','z','intensity') if with_strength else ('x','y','z')
  cloud=read_points_numpy(m,field_names=fields,skip_nans=False).reshape(-1,len(fields))
  cloud=cloud[np.isfinite(cloud[:,:3]).all(axis=1)]
  pts=cloud[:,:3]
  strength=cloud[:,3] if with_strength else None
  # First IMU cloud -> actual optical origin; then gravity align about that origin.
  pts=pts@Rotation.from_quat([s.x,s.y,s.z,s.w]).as_matrix().T+np.array([v.x,v.y,v.z])
  b=body.transform.rotation;v=body.transform.translation
  base_points=pts@Rotation.from_quat([b.x,b.y,b.z,b.w]).as_matrix().T+[v.x,v.y,v.z]
  inside=self.chassis.contains(base_points)
  if self.mesh is not None:inside|=self.mesh.mask(base_points,np.eye(4),self.mesh.nominal_head,.02)
  if not self.rear and self.dual:
   left_self=bottom_left_self_returns(base_points)
   self.left_self_dropped+=int((left_self&~inside).sum())
   inside|=left_self
  self.chassis_dropped+=int(inside.sum())
  finite=np.isfinite(pts).all(axis=1)
  blocked=inside|(np.linalg.norm(pts[:,:2],axis=1)<self.blind)
  near=pts[finite&blocked]
  key=(m.header.stamp.sec,m.header.stamp.nanosec)
  self.near_masks[key]=np.arctan2(near[:,1],near[:,0]).tolist()
  while len(self.near_masks)>100:self.near_masks.pop(next(iter(self.near_masks)))
  weak=np.zeros(len(pts),bool) if self.rear else bottom_right_weak_returns(base_points,strength)
  self.right_weak_dropped+=int((weak&~blocked).sum())
  weak_optical=pts[weak&~blocked]
  pts=pts[finite&~blocked&~weak&(np.ones(len(pts),bool) if self.rear else ~rear_points(pts))]
  level_rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
  optical=pts
  pts=optical@level_rotation.T
  v=t.transform.translation
  if self.clearance: self.clearance.update(m.header,optical,pts,level_rotation,[v.x,v.y,v.z],self.near_masks[key])
  # Do not ray-clear behind an intercepted near-body azimuth or the rear blind sector.
  bins=720
  angle_bins=lambda angles:np.floor((angles+np.pi)*bins/(2*np.pi)).astype(int)%bins
  blocked=np.zeros(bins,bool);blocked[angle_bins(np.asarray(self.near_masks[key]))]=True
  blocked=blocked|np.roll(blocked,1)|np.roll(blocked,-1)
  good=~blocked[angle_bins(np.arctan2(optical[:,1],optical[:,0]))]
  rays=pts[good][::2]
  # Keep independent obstacle marks. Do not use a more distant ray on the
  # same 3-D bearing as the rejected return as evidence of free space.
  rays=without_self_shadow(rays,weak_optical@level_rotation.T,np.zeros(3))
  self.rays[key]=rays+[v.x,v.y,v.z]
  while len(self.rays)>24:self.rays.pop(next(iter(self.rays)))
  tf=TransformStamped();tf.header=m.header;tf.header.frame_id='odom';tf.child_frame_id=self.level;tf.transform.translation=t.transform.translation;tf.transform.rotation.w=1.;self.br.sendTransform(tf)
  h=m.header;h.frame_id=self.level;self.pub.publish(create_cloud_xyz32(h,pts.astype(np.float32)));self.count+=1
  if self.count%50==0:Path(os.environ['ARX_ADAPT_RUN'],f'bridge-{self.source}-health.json').write_text(json.dumps({'frames':self.count,'chassis_points_removed':self.chassis_dropped,'bottom_right_weak_removed':self.right_weak_dropped,'bottom_left_self_removed':self.left_self_dropped,'stale_dropped':self.drops,'stamp':stamp.nanoseconds,'optical_height_m':t.transform.translation.z,'commands_sent':0}))
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
  # Global marks require corroboration; instantaneous local scan stays available.
  try:
   transform=self.buf.lookup_transform("odom",m.header.frame_id,Time.from_msg(m.header.stamp))
  except Exception:
   return
  origin=transform.transform.translation
  xyz=np.array([origin.x,origin.y,origin.z])
  all_points=out+xyz
  all_points=all_points[np.linalg.norm(all_points-xyz,axis=1)<=8.]
  normal=(all_points[:,2]>=.03)&(all_points[:,2]<=1.35)
  hits=all_points[normal]
  seeds=np.asarray(good,dtype=float).reshape(-1,3)+xyz
  rays=self.rays.pop(stamp,None)
  if rays is not None:self.shared_pub.publish(packet(m.header.stamp,xyz,rays,hits,ground=seeds[::2],extra=all_points[~normal]))
  valid=self.global_filter.keep(out,np.array([origin.x,origin.y,origin.z]),stamp[0]+stamp[1]*1e-9)
  self.global_pub.publish(create_cloud_xyz32(m.header,out[valid].astype(np.float32)))

 def safe_ranges(self,m):
  mask=self.near_masks.get((m.header.stamp.sec,m.header.stamp.nanosec))
  out=apply_occlusion(filter_scan(m.ranges,max(self.blind,m.range_min),m.range_max),m.angle_min,m.angle_increment,mask)
  return out if self.rear else mask_scan(out,m.angle_min,m.angle_increment)
 def scan(self,m):
  m.ranges=self.safe_ranges(m);self.scanpub.publish(m)
 def mapping_scan(self,m):
  m.ranges=self.safe_ranges(m);self.mapscanpub.publish(m)
if __name__=='__main__':
 rclpy.init();n=Bridge()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
