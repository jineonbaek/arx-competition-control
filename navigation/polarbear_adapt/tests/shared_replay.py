"""Read actual field and stationary ROS bags without a hardware transport."""
import json,os,sys,time
from collections import defaultdict
from pathlib import Path
import numpy as np
import rosbag2_py as rb
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from sensor_msgs_py.point_cloud2 import read_points_numpy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from shared_voxels import World,SIZE
import stationary_replay
from unittest.mock import Mock

assert os.environ.get('ROS_DOMAIN_ID')=='199'

def read(path,topics):
    r=rb.SequentialReader();r.open(rb.StorageOptions(uri=str(path),storage_id='mcap'),rb.ConverterOptions('',''))
    kinds={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in topics}
    r.set_filter(rb.StorageFilter(topics=topics))
    while r.has_next():
        k,m,t=r.read_next();yield k,deserialize_message(m,kinds[k])

def bottom(path):
    w=World();frames=0;rows=defaultdict(dict);elapsed=[]
    for k,m in read(path,['/pb/level_cloud','/pb/obstacles','/tf','/pb/scan']):
        if k=='/tf':
            ts=[v for v in m.transforms if v.child_frame_id=='pb_lidar_level']
            if not ts:continue
            obj=ts[0]
        else:obj=m
        s=obj.header.stamp.sec*10**9+obj.header.stamp.nanosec
        row=rows[s];row[k]=obj
        if len(row)!=4:continue
        v=row['/tf'].transform.translation;o=np.array([v.x,v.y,v.z]);r=read_points_numpy(row['/pb/level_cloud']).reshape(-1,3)
        h=read_points_numpy(row['/pb/obstacles']).reshape(-1,3)+o
        # Archived level_cloud was gravity-aligned and already blind-masked. Replay finite endpoints only.
        scan=row['/pb/scan']
        h=h[(h[:,2]>=.03)&(h[:,2]<=1.35)&(np.linalg.norm(h-o,axis=1)<=8)]
        start=time.perf_counter();w.update('bottom',s*1e-9,o,r[::2]+o,h);elapsed.append(time.perf_counter()-start)
        frames+=1;rows.pop(s)
        if frames>=140:break
    assert frames>=100 and w.cells and w.ray_removed>0
    return dict(frames=frames,voxels=len(w.cells),ray_removed=w.ray_removed,p95_ms=round(np.percentile(elapsed,95)*1000,1))

def head(path):
    w=World();frames=[];elapsed=[];Original=stationary_replay.Supplement
    class Capture(Original):
        def __init__(self):
            super().__init__()
            self.shared=Mock()
            def got(m):
                a=np.frombuffer(m.data,'<f4').reshape(-1,4);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                start=time.perf_counter();w.update('head',t,a[0,:3],a[a[:,3]==1,:3],a[a[:,3]==2,:3],a[a[:,3]==3,:3],a[a[:,3]==4,:3]);elapsed.append(time.perf_counter()-start)
                frames.append(len(w.cells))
            self.shared.publish.side_effect=got
    stationary_replay.Supplement=Capture
    stationary_replay.run(path)
    assert frames and w.cells
    last=set(w.cells);w.update('bottom',w.last['head']+5,np.zeros(3),np.empty((0,3)),np.empty((0,3)))
    assert set(w.cells)==last
    return dict(frames=len(frames),retained_voxels=len(last),ray_removed=w.ray_removed,p95_ms=round(np.percentile(elapsed,95)*1000,1))

print(json.dumps(dict(bottom=bottom(sys.argv[1]),head=head(sys.argv[2]))))
