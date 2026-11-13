"""A/B on the same time window, plus a warm-up that represents pre-existing false marks."""
import os,sys,time,json
from pathlib import Path
from unittest.mock import patch
import numpy as np
import rosbag2_py as rb
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from shared_voxels import World
assert os.environ.get('ROS_DOMAIN_ID')=='199'
r=rb.SequentialReader();r.open(rb.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rb.ConverterOptions('',''))
r.set_filter(rb.StorageFilter(topics=['/pb/bottom_observations','/pb/head_observations']))
rows=[]
while r.has_next():
 k,v,_=r.read_next();m=deserialize_message(v,PointCloud2);stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
 if rows and stamp-rows[0][0]>float(sys.argv[2]):break
 a=np.frombuffer(m.data,'<f4').reshape(-1,4)
 rows.append((stamp,'head' if 'head' in k else 'bottom',a[0,:3].copy(),a[a[:,3]==1,:3],a[a[:,3]==2,:3],a[a[:,3]==3,:3],a[a[:,3]==4,:3]))
rows.sort(key=lambda row:row[0])
base=World();fixed=World()
base.floor_filter=lambda source,t,origin,rays,hits,ground,extra:hits
times=[]
for t,s,o,r,h,g,e in rows:
 base.update(s,t,o,r,h)
 # Start both maps with the same history, allow the classifier to clear it on further observation.
 if t-rows[0][0]<1.5:
  with patch.object(fixed,'floor_filter',side_effect=lambda source,t,origin,rays,hits,ground,extra:hits):fixed.update(s,t,o,r,h)
 else:
  before=time.perf_counter();fixed.update(s,t,o,r,h,g,e);times.append(time.perf_counter()-before)
center=next(row[2][:2] for row in rows if row[1]=='head')
def cells(w):
 xy=w.projection(t);radius=np.linalg.norm(xy-center,axis=1);return set(map(tuple,np.floor(xy[(radius>.7)&(radius<3.5)]/.05).astype(int)))
a,b=cells(base),cells(fixed)
print(json.dumps(dict(frames=len(rows),area_xy_before=len(a),area_xy_after=len(b),old_only=len(a-b),new_only=len(b-a),
                filtered=fixed.floor_filtered,historic_removed=fixed.floor_removed,tiles=len(fixed.floor.planes),
                processing_ms=(1000*np.percentile(times,[50,95,99])).round(1).tolist())))
assert fixed.floor.planes and sum(fixed.floor_filtered.values())>0
