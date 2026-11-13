"""Isolated STVL test: static wall survives15s dynamic decay. Domain198 only."""
import os
if os.getenv('ROS_DOMAIN_ID')!='198':raise RuntimeError('requires isolated domain198')
import time,signal,subprocess,tempfile,json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from lifecycle_msgs.srv import ChangeState
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
root=Path(__file__).resolve().parents[1];rclpy.init();n=Node('stvl_isolated_fixture');clock=n.create_publisher(Clock,'/clock',10);br=StaticTransformBroadcaster(n)
tf=[]
for parent,child in [('map','odom'),('odom','base_link'),('base_link','bottom_livox')]:
 t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;tf.append(t)
br.sendTransform(tf)
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);mp=n.create_publisher(OccupancyGrid,'/pb/map',q);cp=n.create_publisher(PointCloud2,'/pb/obstacles',10);maps=[];sub=n.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:maps.append(m),q)
m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=.05;m.info.width=m.info.height=200;m.info.origin.position.x=m.info.origin.position.y=-5.;m.info.origin.orientation.w=1.;m.data=[0]*40000
wall=(160,120);m.data[wall[1]*200+wall[0]]=100
state={'t':100.,'hit':True}
def tick():
 state['t']+=.1;c=Clock();c.clock.sec=int(state['t']);c.clock.nanosec=int((state['t']%1)*1e9);clock.publish(c);h=Header();h.frame_id='bottom_livox';h.stamp=c.clock;cp.publish(create_cloud_xyz32(h,[[1.025,1.025,.325]] if state['hit'] else []));m.header.stamp=c.clock;mp.publish(m)
timer=n.create_timer(.02,tick)
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
def wait(f,sec=12):
 end=time.monotonic()+sec
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
 assert f.done(),'timeout';return f.result()
def cell(x,y):
 a=maps[-1];i=int((y-a.info.origin.position.y)/a.info.resolution);j=int((x-a.info.origin.position.x)/a.info.resolution);return a.data[i*a.info.width+j]
with tempfile.TemporaryDirectory() as tmp:
 log=open(Path(tmp)/'node.log','w');child=subprocess.Popen(['/opt/ros/jazzy/lib/nav2_planner/planner_server','--ros-args','--params-file',str(root/'nav2.yaml')],stdout=log,stderr=subprocess.STDOUT)
 try:
  service=n.create_client(ChangeState,'/planner_server/change_state');assert service.wait_for_service(timeout_sec=8)
  for transition in [1,3]:
   req=ChangeState.Request();req.transition.id=transition;assert wait(service.call_async(req)).success
  spin(3);assert maps
  initial_dynamic=cell(1.025,1.025);initial_wall=cell(3.025,1.025);assert initial_dynamic==100,(initial_dynamic,initial_wall)
  state['hit']=False;begin=state['t'];spin(2);early_elapsed=state['t']-begin;early_dynamic=cell(1.025,1.025);assert early_elapsed<15 and early_dynamic==100,(early_elapsed,early_dynamic);spin(2);elapsed=state['t']-begin;after_dynamic=cell(1.025,1.025);after_wall=cell(3.025,1.025)
  assert elapsed>15 and after_dynamic==0 and after_wall==100,(elapsed,after_dynamic,after_wall)
  client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');assert client.wait_for_server(timeout_sec=5)
  goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=True
  goal.start.header.frame_id=goal.goal.header.frame_id='map';goal.start.pose.orientation.w=goal.goal.pose.orientation.w=1.
  goal.goal.pose.position.x=1.;goal.goal.pose.position.y=-1.
  handle=wait(client.send_goal_async(goal));assert handle.accepted
  result=wait(handle.get_result_async());assert result.status==4 and result.result.error_code==0 and len(result.result.path.poses)>2
  print(json.dumps({'synthetic_path_poses':len(result.result.path.poses),'dynamic_before':initial_dynamic,'dynamic_after':after_dynamic,'static_wall_before':initial_wall,'static_wall_after':after_wall,'simulated_elapsed_s':elapsed,'early_elapsed_s':early_elapsed,'early_dynamic':early_dynamic,'commands_sent':0},indent=2))
 finally:
  child.send_signal(signal.SIGINT)
  try:child.wait(timeout=8)
  except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
  log.close()
  if not maps or (maps and cell(1.025,1.025)!=0):print((Path(tmp)/'node.log').read_text()[-6000:])
n.destroy_node();rclpy.shutdown()
