"""Read-only30s sensor-chain snapshot. Raw samples stay on NUC."""
import sys,time,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from tf2_ros import Buffer,TransformListener
from sensor_msgs.msg import PointCloud2,LaserScan
from nav_msgs.msg import OccupancyGrid,Odometry
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs_py.point_cloud2 import read_points_numpy
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
rclpy.init();n=Node('perception_readonly_audit');buf=Buffer(cache_time=Duration(seconds=90));listener=TransformListener(buf,n)
samples={};seen={};maps={};odom=[];scans={};subs=[]
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def receive(m,k):
 if isinstance(m,OccupancyGrid):maps[k]=m;return
 if isinstance(m,Odometry):
  p=m.pose.pose.position;odom.append([ns(m)/1e9,p.x,p.y,p.z]);return
 if isinstance(m,LaserScan):scans.setdefault(k,[]).append(m);return
 bucket=ns(m)//10**9
 if seen.get(k)==bucket:return
 seen[k]=bucket;samples.setdefault(k,[]).append(m)
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
for topic,typ in [('/single/lidar_raw',CustomMsg),('/single/lidar_filtered',CustomMsg),('/single/cloud_body',PointCloud2),('/pb/level_cloud',PointCloud2),('/pb/obstacles',PointCloud2),('/pb/ground',PointCloud2),('/pb/scan_unchecked',LaserScan),('/pb/scan',LaserScan),('/pb/mapping_scan_unchecked',LaserScan),('/pb/mapping_scan',LaserScan),('/pb/map',OccupancyGrid),('/global_costmap/costmap',OccupancyGrid),('/local_costmap/costmap',OccupancyGrid),('/localization/odometry',Odometry)]:
 subs.append(n.create_subscription(typ,topic,lambda m,k=topic:receive(m,k),q if typ==OccupancyGrid else qos_profile_sensor_data))
start=time.monotonic()
while time.monotonic()-start<30:rclpy.spin_once(n,timeout_sec=.02)
arrays={};meta={'samples':{},'scans':{},'maps':{},'errors':[],'commands_sent':0,'duration_s':time.monotonic()-start}
def transform(target,frame,stamp):
 t=buf.lookup_transform(target,frame,stamp).transform;q=t.rotation;v=t.translation
 return Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),np.array([v.x,v.y,v.z])
for topic,msgs in samples.items():
 rows=[]
 for i,m in enumerate(msgs):
  try:
   p=np.asarray([(v.x,v.y,v.z) for v in m.points],np.float32) if isinstance(m,CustomMsg) else read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
   R,t=transform('base_link',m.header.frame_id,Time.from_msg(m.header.stamp));key=topic.replace('/','_')+'_'+str(i);arrays[key]=p@R.T+t;rows.append({'key':key,'stamp_ns':ns(m),'frame':m.header.frame_id,'points':len(p)})
  except Exception as e:meta['errors'].append(topic+':'+str(e))
 meta['samples'][topic]=rows
for topic,msgs in scans.items():
 key=topic.replace('/','_');arrays[key]=np.asarray([m.ranges for m in msgs],np.float32);meta['scans'][topic]={'key':key,'stamp_ns':[ns(m) for m in msgs],'angle_min':msgs[0].angle_min,'angle_increment':msgs[0].angle_increment}
for topic,m in maps.items():
 try:
  R,t=transform(m.header.frame_id,'base_link',Time());key=topic.replace('/','_');arrays[key]=np.asarray(m.data,np.int16).reshape(m.info.height,m.info.width);meta['maps'][topic]={'key':key,'origin':[m.info.origin.position.x,m.info.origin.position.y],'resolution':m.info.resolution,'map_from_body_R':R.tolist(),'map_from_body_t':t.tolist()}
 except Exception as e:meta['errors'].append(topic+':'+str(e))
arrays['odom']=np.asarray(odom);meta['odom_span_m']=np.ptp(arrays['odom'][:,1:],axis=0).tolist() if odom else []
np.savez_compressed(out/'samples.npz',**arrays);(out/'capture.json').write_text(json.dumps(meta,indent=2));print(json.dumps({'capture':str(out),'samples':{k:len(v) for k,v in meta['samples'].items()},'errors':meta['errors'],'odom_span_m':meta['odom_span_m']}));n.destroy_node();rclpy.shutdown()
