"""Isolated stationary fixture for MPPI. No HTTP, SDK or actuator interface."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TransformStamped,TwistStamped,PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from nav2_msgs.action import FollowPath
from lifecycle_msgs.srv import GetState
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--forward',type=float,default=-.22);p.add_argument('--left',type=float,default=0.);p.add_argument('--seconds',type=float,default=4.);a=p.parse_args()
rclpy.init(args=[]);n=Node('mppi_isolated_stationary_fixture');assert __import__('os').getenv('ROS_DOMAIN_ID')=='198'
br=TransformBroadcaster(n);st=StaticTransformBroadcaster(n)
def tr(parent,child,x,y,z):
 t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.translation.x=float(x);t.transform.translation.y=float(y);t.transform.translation.z=float(z);t.transform.rotation.w=1.;return t
static=[tr('base_link','bottom_livox',.18533,-.000003,.230),tr('base_link','rear_livox',-.2832,0,.228)]
static[1].transform.rotation.z=math.sqrt(.5);static[1].transform.rotation.w=math.sqrt(.5);st.sendTransform(static)
odom=n.create_publisher(Odometry,'/localization/odometry',10)
scans=[n.create_publisher(LaserScan,t,10) for t in ['/pb/scan','/pb/head/direct_scan']]
def tick():
 ts=n.get_clock().now().to_msg();t=tr('odom','base_link',0,0,0);t.header.stamp=ts;br.sendTransform(t)
 m=Odometry();m.header.stamp=ts;m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;odom.publish(m)
 for pub,frame in zip(scans,['bottom_livox','rear_livox']):
  m=LaserScan();m.header.stamp=ts;m.header.frame_id=frame;m.angle_min=-math.pi;m.angle_increment=2*math.pi/720;m.angle_max=m.angle_min+719*m.angle_increment;m.range_min=.35;m.range_max=8.;m.ranges=[8.]*720;pub.publish(m)
n.create_timer(.05,tick);rows=[]
n.create_subscription(TwistStamped,'/arx_mppi_trial/raw_velocity',lambda m:rows.append([time.monotonic(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]),100)
cl=ActionClient(n,FollowPath,'/arx_mppi_trial/follow_path')
def pump(test,limit):
 end=time.monotonic()+limit
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.02)
  if test():return True
 return False
state=n.create_client(GetState,'/arx_mppi_trial/controller_server/get_state')
assert pump(state.service_is_ready,25.),'lifecycle unavailable'
deadline=time.monotonic()+25.
while time.monotonic()<deadline:
 f=state.call_async(GetState.Request());assert pump(f.done,3.)
 if f.result().current_state.id==3 and cl.server_is_ready():break
 pump(lambda:False,.2)
else:raise RuntimeError('controller not active')
pump(lambda:False,.5)
for trial in range(3):
 rows.clear();path=NavPath();path.header.frame_id='odom';path.header.stamp=n.get_clock().now().to_msg()
 for i in range(18):
  pose=PoseStamped();pose.header=path.header;pose.pose.position.x=a.forward*i/17;pose.pose.position.y=a.left*i/17;pose.pose.orientation.w=1.;path.poses.append(pose)
 request=FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id='goal_checker',progress_checker_id='progress_checker')
 sent=cl.send_goal_async(request);assert pump(sent.done,4.);handle=sent.result();assert handle.accepted
 result=handle.get_result_async();pump(result.done,a.seconds)
 if not result.done():cancel=handle.cancel_goal_async();assert pump(cancel.done,3.)
 assert pump(result.done,3.);pump(lambda:False,.3)
 output=a.output.with_name(a.output.stem+f'-{trial+1}.json');output.write_text(json.dumps(dict(stationary_fixture=True,physical_commands=0,result_status=result.result().status,error_code=result.result().result.error_code,raw=rows)))
 assert len(rows)>10 and max(abs(v) for v in rows[-1][1:])<1e-8
 print(output,flush=True)
n.destroy_node();rclpy.shutdown()
