"""Run the production voxel rules on simultaneous real packets, with a labelled walker overlay."""
import os,sys,time,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import rosbag2_py as rb
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from shared_voxels import World,SIZE
assert os.environ.get('ROS_DOMAIN_ID')=='199'
r=rb.SequentialReader();r.open(rb.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rb.ConverterOptions('',''))
r.set_filter(rb.StorageFilter(topics=['/pb/bottom_observations','/pb/head_observations']))
rows=[]
while r.has_next():
    topic,v,arrival=r.read_next();m=deserialize_message(v,PointCloud2);a=np.frombuffer(m.data,'<f4').reshape(-1,4)
    rows.append(('bottom' if 'bottom' in topic else 'head',m.header.stamp.sec+1e-9*m.header.stamp.nanosec,a[0,:3].copy(),a[a[:,3]==1,:3],a[a[:,3]==2,:3]))
rows.sort(key=lambda row:row[1])
static=np.vstack([h[::3,:2] for _,_,_,_,h in rows if len(h)])
tree=cKDTree(static)
seeds=np.array([[x,y] for x in np.arange(-2.5,1.,.4) for y in np.arange(-2.5,2.5,.4)])
offset=np.linspace(0,1.6,17)
scores=[tree.query(np.c_[s[0]+offset,np.full(len(offset),s[1])])[0].min() for s in seeds]
start=seeds[np.argmax(scores)]
assert max(scores)>.38, max(scores)
w=World();baseline=World();times=[];t0=rows[0][1];leaks=[];counts={'bottom':0,'head':0}
shape=np.array([[x,y,z] for x in np.arange(-.15,.151,.05) for y in (-.06,0.,.06) for z in np.arange(.15,.81,.075)])
for source,t,o,rays,hits in rows:
    baseline.update(source,t,o,rays,hits)
    age=t-t0
    shift=np.clip(.65*(age-2),0,1.6);centre=start+[shift,0]
    actor=shape+[*centre,0.]
    # Some vacated heights receive actual crossing rays in the fixture; others must be retired by ownership.
    prevkeys=[k for k,c in w.cells.items() if c.owner and abs((k[1]+.5)*SIZE[1]-start[1])<.16 and
              start[0]-.22<(k[0]+.5)*SIZE[0]<centre[0]-.21 and k[2]%3==0]
    extra=np.asarray([o+2*((np.asarray(k)+.5)*SIZE-o) for k in prevkeys]).reshape(-1,3)
    before=time.perf_counter();w.update(source,t,o,np.vstack((rays,extra)),np.vstack((hits,actor)))
    times.append(time.perf_counter()-before);counts[source]+=1
    if 5.5<age<7:
        leaks.append(sum(abs((k[1]+.5)*SIZE[1]-start[1])<.15 and start[0]-.2<(k[0]+.5)*SIZE[0]<centre[0]-.3 for k in w.cells))
assert min(counts.values())>30 and w.track_removed>baseline.track_removed and leaks and max(leaks)==0,(counts,max(leaks))
print(json.dumps(dict(frames=counts,fixture_trail_max=int(max(leaks)),baseline_track_retired=baseline.track_removed,
                      fixture_track_retired=w.track_removed,p95_ms=round(1000*np.percentile(times,95),1),fixture_location=start.tolist())))
