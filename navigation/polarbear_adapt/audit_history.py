"""Read recorded body clouds and odometry; distinguish body-fixed near returns."""
import sys,json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py.point_cloud2 import read_points_numpy
root=Path(sys.argv[1]);out=Path(sys.argv[2]);r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));types={t.name:get_message(t.type) for t in r.get_all_topics_and_types()};r.set_filter(rosbag2_py.StorageFilter(topics=['/single/cloud_body','/localization/odometry']));lastbin=None;lastpose=None;speed=0.;rows=[];hist=np.zeros((30,30,30),int);moving=0
imu=np.array([.196330,.023287,.185880]);opt=np.array([.185330,-.000003,.230])
while r.has_next():
 topic,b,_=r.read_next();m=deserialize_message(b,types[topic]);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if topic=='/localization/odometry':
  p=m.pose.pose.position;p=np.array([p.x,p.y,p.z])
  if lastpose and t>lastpose[0]:speed=np.linalg.norm(p[:2]-lastpose[1][:2])/(t-lastpose[0])
  lastpose=(t,p);continue
 bucket=int(t/2)
 if bucket==lastbin:continue
 lastbin=bucket;a=read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)+imu;d=np.linalg.norm(a[:,:2],axis=1);near=a[(d<.529)&(a[:,2]>=.03)&(a[:,2]<=1.35)];outside=near[np.linalg.norm((near-opt)[:,:2],axis=1)>=.35];rows.append([t,float(speed),len(near),len(outside)])
 if speed>.05:
  moving+=1;idx=np.floor((near+np.array([.6,.6,0]))/.05).astype(int);idx=idx[(idx>=0).all(axis=1)&(idx<30).all(axis=1)]
  for v in set(map(tuple,idx)):hist[v]+=1
rows=np.array(rows);indices=np.argwhere(hist>0);rank=sorted(indices,key=lambda i:hist[tuple(i)],reverse=True)[:12];result={'sampled_frames':len(rows),'moving_frames_speed_over005':moving,'near_points_quantiles':np.quantile(rows[:,2],[0,.5,.95,1]).tolist(),'near_points_after035_quantiles':np.quantile(rows[:,3],[0,.5,.95,1]).tolist(),'frames_any_near_after035_fraction':float(np.mean(rows[:,3]>0)),'persistent_near_voxels_during_translation':[{'body_xyz_m':((i+.5)*.05-np.array([.6,.6,0])).tolist(),'frame_fraction':float(hist[tuple(i)]/max(1,moving))} for i in rank],'note':'body-fixed persistence is evidence of self/attached returns, not proof of cable identity; cloud_body is already LIO downsampled and CAD filtered','commands_sent':0};out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
