"""Domain 199: inspect the actual Nav2 plugin + inflation as both sensors add/clear a fixture."""
import os,sys,time,numpy as np
assert os.environ.get('ROS_DOMAIN_ID')=='199'
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from nav_msgs.msg import OccupancyGrid,GridCells
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile,DurabilityPolicy
from shared_obstacles import packet
from shared_voxels import SIZE

rclpy.init();n=rclpy.create_node('shared_fixture')
pub={k:n.create_publisher(PointCloud2,'/pb/'+k+'_observations',1) for k in ['bottom','head']}
maps={};empty_at=[]
def empty_seen(m):
    if not m.width:empty_at.append(time.monotonic())
n.create_subscription(PointCloud2,'/pb/shared_obstacles',empty_seen,1)
for k in ('local','global'):
    n.create_subscription(OccupancyGrid,'/'+k+'_costmap/costmap',lambda m,k=k:maps.update({k:m}),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))

origin=np.array([0.,0.,.23]);empty=np.empty((0,3))
p=np.array([[1.025,y,.3375] for y in (.025,.075,.125)])
r=origin+2*(p-origin)
def feed(phase,duration):
    deadline=time.monotonic()+duration;sent=0
    while time.monotonic()<deadline:
        if time.monotonic()>sent:
            ts=n.get_clock().now().to_msg()
            pub['bottom'].publish(packet(ts,origin,r if phase=='clear' else empty,empty))
            if phase=='mark':pub['head'].publish(packet(ts,origin,p,p))
            sent=time.monotonic()+.094
        rclpy.spin_once(n,timeout_sec=.015)
def read():
    result={}
    for k,m in maps.items():
        o=m.info.origin.position
        ij=np.floor((np.array([[1.025,.075],[1.225,.075]])-[o.x,o.y])/m.info.resolution).astype(int)
        result[k]=np.array(m.data).reshape(m.info.height,m.info.width)[ij[:,1],ij[:,0]].tolist()
    assert len(result)==2
    return result
deadline=time.monotonic()+10
while len(maps)<2 and time.monotonic()<deadline:feed('mark',.2)
feed('mark',.8);a=read();assert all(x[0]==100 and x[1]>0 for x in a.values()),a
feed('miss',2);b=read();assert a==b,(a,b)
empty_at.clear();start=time.monotonic()
while True:
    feed('clear',.06);c=read()
    if all(x==[0,0] for x in c.values()):break
    assert time.monotonic()-start<2,(a,b,c)
print('actual Nav2 master grids, mark/inflation',a,'after head stops',b,'cross-sensor clear',c,'clear_seconds',round(time.monotonic()-start,3),'after_observed_clear',round(time.monotonic()-empty_at[0],3))
assert time.monotonic()-empty_at[0]<.40
rclpy.shutdown()
