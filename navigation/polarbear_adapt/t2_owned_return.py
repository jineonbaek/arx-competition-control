"""Read-only ROS pose/controller worker. Emits SDK candidates; never owns hardware."""
import json,math,sys,time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.qos import QoSProfile,DurabilityPolicy
from chassis_pose import valid_pose,planar
from chassis_pose_policy import MOTION_MAX_AGE_NS
from task_approach import TaskApproach,TaskStopReview
from t2_arrival import T2Arrival
from mppi_trial.stationary import StopWindow

def valid_batch(msg,now_ns):
 if msg is None or msg.header.frame_id!='map' or len(msg.poses)!=2 or not all(valid_pose(p) for p in msg.poses):return False
 stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
 return 0<=now_ns-stamp<=MOTION_MAX_AGE_NS

class ReturnControl:
 def __init__(self,goal):
  self.point=dict(id='t2-timeout-return',type='task',point_id='point_2',pose=list(goal));self.policy=T2Arrival(goal);self.chain=None;self.review=TaskStopReview(self.point,self.policy);self.previous=np.zeros(3);self.source=None;self.stop=None
 def update(self,msg,now):
  if not valid_batch(msg,int(now*1e9)):
   self.chain=None;self.stop=None;self.source=None;self.previous=np.zeros(3);return dict(status='waiting',command=[0.,0.,0.],reason='等待新鲜权威车位')
  mp,od=(np.asarray(planar(p),float) for p in msg.poses);source=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
  if self.chain is None:
   self.chain=TaskApproach(mp,od,[self.point],arrival_policy=self.policy);self.chain.previous_sample=(source,mp.copy(),od.copy());self.source=source
  command=self.previous
  if source!=self.source:
   dt=source-self.source
   if not 0<dt<=1.:
    self.chain=None;self.stop=None;self.previous=np.zeros(3);return dict(status='waiting',command=[0.,0.,0.],reason='等待连续权威车位')
   step=self.chain.step(mp,od,source,now,min(.1,dt),self.previous);self.source=source;command=np.asarray(step.command)
   if step.terminal_near and self.stop is None:self.stop=StopWindow(after=now)
   if self.stop is not None:
    command=np.zeros(3);stationary=self.stop.add(source,od)
    if stationary:
     verdict=self.review.check(mp,True)
     if verdict in ('arrived','t2_fallback'):return dict(status='completed',command=[0.,0.,0.],source_ns=int(source*1e9),map_pose=mp.tolist(),odom_pose=od.tolist(),arrival=verdict,stationary=True)
     if verdict=='park':raise ValueError('T2回位精修未确认，不能放行后续导航')
     self.chain=TaskApproach(mp,od,[self.point],arrival_policy=self.policy);self.chain.previous_sample=(source,mp.copy(),od.copy());self.stop=None
  self.previous=command
  return dict(status='running',command=list(map(float,command)),source_ns=int(source*1e9),map_pose=mp.tolist(),odom_pose=od.tolist())

def main():
 goal=json.loads(sys.argv[1]);assert len(goal)==3 and np.isfinite(goal).all()
 rclpy.init();node=rclpy.create_node('t2_task_owned_return_pose');latest=[None]
 controller=ReturnControl(goal);reject_before=[0]
 def receive(msg):
  now=node.get_clock().now().nanoseconds;stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
  if not valid_batch(msg,now):reject_before[0]=now;latest[0]=None
  elif stamp>reject_before[0]:latest[0]=msg
 node.create_subscription(PoseArray,'/pb/chassis_pose',receive,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 try:
  for line in sys.stdin:
   if json.loads(line).get('action')!='step':break
   for _ in range(3):rclpy.spin_once(node,timeout_sec=.008)
   try:out=controller.update(latest[0],node.get_clock().now().nanoseconds/1e9)
   except Exception as e:out=dict(status='failed',command=[0.,0.,0.],reason=str(e))
   print(json.dumps(out,allow_nan=False),flush=True)
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
