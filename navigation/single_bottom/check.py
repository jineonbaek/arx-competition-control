"""Bounded read-only rate/pose checks; does not publish commands."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
qos_profile_sensor_data=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import Imu,PointCloud2,LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from livox_ros_driver2.msg import CustomMsg
p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--seconds',type=float,default=30);a=p.parse_args();rclpy.init();node=Node('single_bottom_readonly_check');rows={};subs=[];extra={}
spec=[('/single/imu',Imu),('/single/lidar_raw',CustomMsg),('/single/lidar_filtered',CustomMsg),('/single/odometry_raw',Odometry),('/localization/odometry',Odometry),('/mapping/cloud_map',PointCloud2),('/mapping/grid',OccupancyGrid),('/single/scan',LaserScan),('/amcl_pose',PoseWithCovarianceStamped)]
def receive(m,key):
 rows[key].append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
 if isinstance(m,Odometry):
  v=m.pose.pose;extra.setdefault(key,[]).append([v.position.x,v.position.y,v.position.z,v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w])
 elif isinstance(m,LaserScan):extra[key]={'finite_ranges':int(np.isfinite(m.ranges).sum()),'beams':len(m.ranges),'frame':m.header.frame_id}
 elif isinstance(m,OccupancyGrid):extra[key]={'width':m.info.width,'height':m.info.height,'free':sum(x==0 for x in m.data),'occupied':sum(x==100 for x in m.data),'frame':m.header.frame_id}
 elif isinstance(m,PoseWithCovarianceStamped):extra[key]={'frame':m.header.frame_id,'position':[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z],'covariance':list(m.pose.covariance)}
for topic,typ in spec:
 rows[topic]=[];subs.append(node.create_subscription(typ,topic,lambda m,k=topic:receive(m,k),qos_profile_sensor_data))
start=time.monotonic()
while time.monotonic()-start<a.seconds:rclpy.spin_once(node,timeout_sec=.05)
result={'duration_s':time.monotonic()-start,'commands_sent':0,'streams':{}}
for topic,ts in rows.items():
 d=np.diff(np.array(ts,np.int64))/1e9;item={'count':len(ts)}
 if len(d):item.update(hz=float(len(d)/((ts[-1]-ts[0])/1e9)) if ts[-1]>ts[0] else None,max_gap_s=float(d.max()),nonincreasing=int((d<=0).sum()))
 if isinstance(extra.get(topic),list):
  x=np.array(extra[topic]);item.update(pose_finite=bool(np.isfinite(x).all()),position_span_mm=(np.ptp(x[:,:3],axis=0)*1000).tolist(),last_pose=x[-1].tolist())
 else:item['latest']=extra.get(topic)
 result['streams'][topic]=item
Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));node.destroy_node();rclpy.shutdown()
