"""Synthetic Nav2 integration ONLY on isolated ROS_DOMAIN_ID=199. No robot commands."""
import os,time,json,tempfile,subprocess,signal
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='199':raise RuntimeError('Requires isolated domain199')
import yaml,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.action import ActionClient
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
root=Path(__file__).resolve().parents[1];params=root/'nav2.yaml';result=[]
rclpy.init();n=Node('isolated_preview_fixture');br=StaticTransformBroadcaster(n)
transforms=[]
for parent,child in [('preview_map','odom'),('odom','base_link'),('base_link','bottom_livox')]:
 t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;transforms.append(t)
br.sendTransform(transforms)
pub=n.create_publisher(LaserScan,'/preview/scan',10);maps=[]
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
sub=n.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:maps.append(m),q)
fixture={'range':.325,'full':False}
def scan():
 m=LaserScan();m.header.frame_id='bottom_livox';m.header.stamp=n.get_clock().now().to_msg();m.angle_min=-3.141592653589793;m.angle_increment=2*3.141592653589793/720;m.angle_max=m.angle_min+719*m.angle_increment;m.range_min=.2;m.range_max=8.;m.scan_time=.1
 m.ranges=[fixture['range']]*720 if fixture['full'] else [float('nan')]*720;m.ranges[360]=fixture['range'];pub.publish(m)
timer=n.create_timer(.1,scan)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
def wait(f,seconds=10):
 end=time.monotonic()+seconds
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 assert f.done(),'service/action timeout';return f.result()
def cost(x,y):
 m=maps[-1];i=int((y-m.info.origin.position.y)/m.info.resolution);j=int((x-m.info.origin.position.x)/m.info.resolution);return m.data[i*m.info.width+j]
with tempfile.TemporaryDirectory() as tmp:
 log=open(Path(tmp)/'planner.log','w');process=subprocess.Popen(['/opt/ros/jazzy/lib/nav2_planner/planner_server','--ros-args','--params-file',str(params)],stdout=log,stderr=subprocess.STDOUT)
 try:
  client=n.create_client(ChangeState,'/planner_server/change_state');assert client.wait_for_service(timeout_sec=10)
  for transition in [1,3]:
   req=ChangeState.Request();req.transition.id=transition;assert wait(client.call_async(req)).success
  spin(3);assert maps
  first=cost(.325,0);assert first==100,('near obstacle erased',first)
  fixture['range']=float('nan');spin(2);still=cost(.325,0);assert still==100,('NaN incorrectly cleared',still)
  fixture['range']=2.;spin(2);cleared=cost(.325,0);assert cleared==0,('finite ray did not clear',cleared)
  unknown=cost(-3.,-3.);assert unknown==-1,('unobserved became free',unknown)
  fixture.update(range=4.,full=True);spin(3)
  action=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');assert action.wait_for_server(timeout_sec=5)
  goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=True
  for pose,x in [(goal.start,1.),(goal.goal,2.)]:pose.header.frame_id='preview_map';pose.pose.position.x=x;pose.pose.orientation.w=1.
  handle=wait(action.send_goal_async(goal));assert handle.accepted
  reply=wait(handle.get_result_async());assert reply.status==4 and len(reply.result.path.poses)>5,(reply.status,reply.result.error_code)
  result={'near_obstacle_preserved':first,'nan_preserved':still,'finite_ray_cleared':cleared,'unobserved_unknown':unknown,'synthetic_path_poses':len(reply.result.path.poses),'commands_sent':0,'domain':199}
  print(json.dumps(result,indent=2))
 finally:
  process.send_signal(signal.SIGINT)
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
  log.close()
  if not result:print((Path(tmp)/'planner.log').read_text()[-6000:])
n.destroy_node();rclpy.shutdown()
