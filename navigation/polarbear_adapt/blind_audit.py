"""Recorded scan blind-radius comparison. No scene export or robot publisher."""
import argparse,json
from pathlib import Path
import numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('output');a=p.parse_args()
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=a.bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/preview/scan']))
thresholds=[.2,.25,.3,.35,.4,.45,.5,.55,.6];stats={str(v):[] for v in thresholds};ranges=[];bearings=[]
while r.has_next():
 _,data,t=r.read_next();m=deserialize_message(data,LaserScan);d=np.array(m.ranges);ang=m.angle_min+np.arange(len(d))*m.angle_increment;valid=np.isfinite(d);base=np.hypot(.18533+d*np.cos(ang),-.000003+d*np.sin(ang))
 for v in thresholds:
  keep=valid&(d>=v);near=keep&(base<.529)
  stats[str(v)].append([t/1e9,int(keep.sum()),int(near.sum()),float(base[keep].min()) if keep.any() else None,int((valid&(d<v)).sum())])
 # Body-adjacent persistent echoes are not automatically classified as selfpoints.
 if len(ranges)<2000:ranges.extend(d[valid&(base<.25)].tolist());bearings.extend(ang[valid&(base<.25)].tolist())
summary={'warning':'Filtering removes observations, does not prove true free space or identify selfpoints. Scan range is horizontal distance from optical origin.', 'candidates':{}}
for k,rows in stats.items():
 x=np.array(rows,float);summary['candidates'][k]={'frames':len(x),'retained_beams_median':float(np.median(x[:,1])),'frames_with_return_in_collision_radius_fraction':float(np.mean(x[:,2]>0)),'near_beams_q0_50_95_100':np.quantile(x[:,2],[0,.5,.95,1]).tolist(),'nearest_base_distance_q0_50_95_100':np.nanquantile(x[:,3],[0,.5,.95,1]).tolist(),'removed_beams_median':float(np.median(x[:,4]))}
Path(a.output).write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
