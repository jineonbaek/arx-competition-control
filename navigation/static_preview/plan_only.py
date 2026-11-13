"""RViz Publish Point -> ComputePathToPose only. No NavigateToPose or Twist."""
import json,os,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped,PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import PointCloud2,LaserScan
from nav2_msgs.action import ComputePathToPose
from core import valid_goal
class Planner(Node):
 def __init__(self):
  super().__init__('preview_plan_only');self.odom=None;self.ot=0.;self.ct=0.;self.st=0.;self.target=None;self.last_request=0.;self.busy=False;self.seq=0;self.goal_handle=None
  self.pub=self.create_publisher(NavPath,'/preview/path',1)
  self.client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
  self.subs=[self.create_subscription(Odometry,'/localization/odometry',self.onodom,10),self.create_subscription(PointCloud2,'/preview/level_cloud',self.oncloud,qos_profile_sensor_data),self.create_subscription(PointStamped,'/clicked_point',self.goal,10)]
  self.scan_sub=self.create_subscription(LaserScan,'/preview/scan',self.onscan,qos_profile_sensor_data)
  self.timer=self.create_timer(.2,self.watch)
 def fresh(self):
  now=self.get_clock().now().nanoseconds/1e9
  return self.odom is not None and 0<=now-self.st<1. and valid_goal([0,0,0],now-self.ct,now-self.ot,'preview_map')
 def onscan(self,m):self.st=m.header.stamp.sec+m.header.stamp.nanosec/1e9
 def onodom(self,m):self.odom=m;self.ot=m.header.stamp.sec+m.header.stamp.nanosec/1e9
 def oncloud(self,m):self.ct=m.header.stamp.sec+m.header.stamp.nanosec/1e9
 def report(self,status,**kw):
  data={'status':status,'time':time.time(),'commands_sent':0,**kw};p=Path(os.environ['ARX_PREVIEW_RUN']);(p/'plan-status.json').write_text(json.dumps(data,indent=2))
  with (p/'plan-events.jsonl').open('a') as f:f.write(json.dumps(data)+'\n')
  self.get_logger().info(json.dumps(data))
 def clear(self):
  p=NavPath();p.header.frame_id='preview_map';p.header.stamp=self.get_clock().now().to_msg();self.pub.publish(p)
 def watch(self):
  if not self.fresh():
   self.clear()
   if self.goal_handle:self.goal_handle.cancel_goal_async();self.goal_handle=None
  elif self.target is not None and not self.busy and time.monotonic()-self.last_request>2.:self.goal(self.target)
 def goal(self,m):
  p=[m.point.x,m.point.y,m.point.z];now=self.get_clock().now().nanoseconds/1e9
  if self.busy:self.report('rejected_busy');return
  self.clear();self.target=None
  if not self.fresh() or not valid_goal(p,now-self.ct,now-self.ot,m.header.frame_id):self.report('rejected_stale_or_frame_or_nonfinite');return
  if not self.client.server_is_ready():self.report('rejected_planner_not_ready');return
  g=ComputePathToPose.Goal();g.planner_id='GridBased';g.use_start=False;g.goal=PoseStamped();g.goal.header.frame_id='preview_map';g.goal.header.stamp=self.get_clock().now().to_msg();g.goal.pose.position.x=p[0];g.goal.pose.position.y=p[1];g.goal.pose.orientation.w=1.
  self.target=m;self.last_request=time.monotonic();self.busy=True;self.started=time.monotonic();self.report('requested',target=p)
  self.client.send_goal_async(g).add_done_callback(self.accepted)
 def accepted(self,f):
  try:
   self.goal_handle=f.result()
   if not self.goal_handle.accepted:self.busy=False;self.report('rejected_by_planner');return
   self.goal_handle.get_result_async().add_done_callback(self.done)
  except Exception as e:self.busy=False;self.report('error',detail=str(e))
 def done(self,f):
  self.busy=False;self.goal_handle=None
  try:
   result=f.result();r=result.result
   if result.status==4 and r.error_code==0 and r.path.poses and self.fresh():self.pub.publish(r.path);self.report('path_ready',poses=len(r.path.poses),duration_s=time.monotonic()-self.started)
   else:self.clear();self.report('no_valid_path',action_status=result.status,error_code=r.error_code)
  except Exception as e:self.clear();self.report('error',detail=str(e))
if __name__=='__main__':
 rclpy.init();n=Planner()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
