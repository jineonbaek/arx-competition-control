"""Explicit saved-map initial pose with nonzero uncertainty; no motion command."""
import os,time,math,json
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import SetInitialPose
from core import localization_initial_pose,localization_seed_covariance

def main():
 x,y,yaw=localization_initial_pose(os.environ);rclpy.init();node=rclpy.create_node('explicit_localization_seed');state=node.create_client(GetState,'/amcl/get_state');seed=node.create_client(SetInitialPose,'/set_initial_pose');deadline=time.monotonic()+45;active=False
 try:
  while time.monotonic()<deadline:
   if not state.wait_for_service(timeout_sec=1):continue
   f=state.call_async(GetState.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=2)
   if f.done() and f.result().current_state.id==3:active=True;break
  if not active or not seed.wait_for_service(timeout_sec=5):raise RuntimeError('AMCL not active; no initial pose sent')
  req=SetInitialPose.Request();req.pose.header.frame_id='map';req.pose.header.stamp=node.get_clock().now().to_msg();req.pose.pose.pose.position.x=x;req.pose.pose.pose.position.y=y;req.pose.pose.pose.orientation.z=math.sin(yaw/2);req.pose.pose.pose.orientation.w=math.cos(yaw/2);req.pose.pose.covariance=localization_seed_covariance()
  f=seed.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=5)
  if not f.done() or f.exception():raise RuntimeError('AMCL initial pose service did not respond')
  run=Path(os.environ['ARX_SINGLE_RUN']);(run/'localization-seed.json').write_text(json.dumps({'x':x,'y':y,'yaw':yaw,'covariance':localization_seed_covariance(),'note':'explicit initial search distribution, not localization accuracy','commands_sent':0},indent=2));node.get_logger().info('Explicit map pose seeded with 0.1m / 10deg initial standard deviations')
 finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
