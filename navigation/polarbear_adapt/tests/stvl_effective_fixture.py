import json,os,sys,time
if os.getenv("ROS_DOMAIN_ID")!="181":
    raise SystemExit("Run this fixture only in isolated ROS domain 181")
from pathlib import Path
import numpy as np,rclpy
from rclpy.qos import DurabilityPolicy,QoSProfile,qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy,create_cloud_xyz32
from std_msgs.msg import Header
from lifecycle_msgs.srv import ChangeState
from shared_obstacles import packet
d=Path(sys.argv[1]);rclpy.init();node=rclpy.create_node('stvl_two_height_fixture')
clock=node.get_clock();q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
tf=StaticTransformBroadcaster(node)
ts=[]
for child in ('base_link','pb_nav_bottom_sensor','pb_nav_head_sensor'):
 t=TransformStamped();t.header.frame_id='map';t.child_frame_id=child;t.transform.rotation.w=1.
 if child!='base_link':t.transform.translation.x=.025;t.transform.translation.y=.025;t.transform.translation.z=.275
 ts.append(t)
tf.sendTransform(ts)
mp=node.create_publisher(OccupancyGrid,'/pb/map',q);m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=.05;m.info.width=m.info.height=180;m.info.origin.position.x=m.info.origin.position.y=-4.0;m.info.origin.orientation.w=1.;m.data=[0]*32400;mp.publish(m)
sources={s:node.create_publisher(PointCloud2,f'/pb/{s}/stvl_evidence',10) for s in ('bottom','head')}
frustum={s:node.create_publisher(PointCloud2,f'/pb/{s}/nav_mark',3) for s in sources}
grids=[];vox=[]
node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:grids.append(m),q)
node.create_subscription(PointCloud2,'/global_costmap/voxel_grid',lambda m:vox.append(read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)),qos_profile_sensor_data)
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def send(s,hits=(),rays=(),stamp=None):
 o=np.array([.025,.025,.275]);stamp=clock.now().to_msg() if stamp is None else stamp
 h=np.asarray(hits).reshape(-1,3);r=np.asarray(rays).reshape(-1,3)
 p=packet(stamp,o,r,h);p.header.frame_id='map';sources[s].publish(p)
 frustum[s].publish(create_cloud_xyz32(Header(frame_id='map',stamp=stamp),h.astype('f4')))
def vf(point):
 a=vox[-1];return bool((np.max(np.abs(a-point),axis=1)<.015).any()) if len(a) else False
def cost(pt):
 g=grids[-1];x,y=np.floor((pt[:2]-[g.info.origin.position.x,g.info.origin.position.y])/g.info.resolution).astype(int)
 return g.data[int(y*g.info.width+x)]
c=node.create_client(ChangeState,'/global_costmap/global_costmap/change_state');assert c.wait_for_service(timeout_sec=8)
for sid in [1,3]:
 req=ChangeState.Request();req.transition.id=sid;f=c.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=8);assert f.result().success
spin(.5)
low=np.array([1.025,.025,.275]);high=low+[0,0,.5];other=np.array([1.025,1.025,.275]);hidden=np.array([-1.025,1.025,.275]);acc=np.array([2.025,.025,.275])
# Separate heights in one XY; isolated head-only cell; a head-only hidden cell.
ts=clock.now().to_msg();t0=time.monotonic()
send('head',[low,high,other,hidden,acc],stamp=ts);spin(.25)
assert all(vf(p) for p in (low,high,other,hidden,acc)),(len(vox),len(grids))
o=np.array([.025,.025,.275]);ray=o+2*(low-o);ray2=o+2*(other-o)
for _ in range(3): send('bottom',rays=[ray,ray2]);spin(.09)
send('bottom',rays=[[.025,2.025,.275]]);spin(.09);assert vf(low) and vf(other)
send('bottom',rays=[ray,ray2]);spin(.22)
assert not vf(low) and not vf(other) and vf(high) and vf(hidden) and cost(low)==100 and cost(other)<100
early=time.monotonic()-t0
# Two look-throughs on each sensor are not four on either. Then split the
# first sensor's positives around one endpoint to exercise 4/5 specifically.
ratio=np.array([1.025,-.975,.275]); rr=o+2*(ratio-o)
send('head',[ratio]);spin(.12)
for sid in ('bottom','head','bottom','head'):send(sid,rays=[rr]);spin(.07)
assert vf(ratio)
send('bottom',rays=[ratio]);spin(.07)
send('bottom',rays=[rr]);spin(.07)
assert vf(ratio)
send('bottom',rays=[rr]);spin(.20)
assert not vf(ratio)
# A delayed mark predating the clear must not resurrect the low voxel.
send('head',[low],stamp=ts);spin(.15);assert not vf(low)
send('bottom',[low]);spin(.22);assert vf(low) and vf(high)
# One later passing ray is insufficient for 4-vote clear, but inside a real
# fresh lidar frustum it accelerates this 5.0-decay cell.
while time.monotonic()-t0<3.3:send('bottom');spin(.1)
send('bottom',rays=[o+2*(acc-o)]);spin(.22)
assert not vf(acc), 'visible acceleration did not clear'
while time.monotonic()-t0<14.5:send('bottom');spin(.10)
assert vf(hidden),'hidden head voxel cleared early'
while time.monotonic()-t0<15.8:send('bottom');spin(.10)
assert not vf(hidden) and cost(hidden)<100
# Outside the acceleration frustum, cross an unseen time interval and split
# another voxel's streak with a same-source endpoint.
a=np.array([-.975,-.125,.275]); b=np.array([-.975,.325,.275])
ra=o+2*(a-o); rb=o+2*(b-o)
send('head',[a,b]);spin(.22)
for _ in range(2):send('bottom',rays=[ra,rb]);spin(.10)
spin(1.25)
send('bottom',rays=[b]);spin(.10)
send('bottom',rays=[ra,rb]);spin(.12)
assert vf(a) and vf(b)
send('bottom',rays=[ra,rb]);spin(.20)
assert not vf(a) and vf(b), 'four valid look-throughs survive an unobserved gap; a hit breaks the streak'
for _ in range(2):send('bottom',rays=[rb]);spin(.12)
assert not vf(b)
print(json.dumps(dict(cross_second_gap=True,hit_breaks_streak=True,ratio_pass=True,sources_independent=True,early_clear_elapsed_s=round(early,3),table_edge_held=True,head_pause_aged=True,
    stale_did_not_remark=True,fresh_remark=True,visible_acceleration=True,hidden_present_at_14_5s=True,hidden_cleared_by_15_8s=True)))
