"""Short source gaps park output, retain native action, never use stale proof."""
import copy,time,unittest
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np,rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseArray,PoseStamped,TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool,String
import test_mission_ros as f

class ShortInputTracking(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init());tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.MissionROS.setUp;tearDown=f.MissionROS.tearDown;refresh=f.MissionROS.refresh;add=f.MissionROS.add
 def goal(self):
  self.add('stop',2.);self.n.waypoints.start(None);self.n.seen=self.n.seq
  self.n.route=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]]);self.n.route_time=self.n.now()
 def test_bounded_pump_uses_ready_pose_before_next_proof(self):
  self.n.destroy_timer(self.timer);old=copy.deepcopy(self.n.pose_snapshot);old.header.stamp=Time(nanoseconds=self.n.get_clock().now().nanoseconds-500000000).to_msg();self.n.receive_chassis_snapshot(old)
  fresh=copy.deepcopy(old);fresh.header.stamp=self.n.get_clock().now().to_msg();queue=[lambda:time.sleep(.003),lambda:self.n.receive_chassis_snapshot(fresh)]
  def one(*args,**kw):
   if queue:queue.pop(0)()
  with patch('rclpy.spin_once',one):self.n.spin(.001)
  self.assertEqual(queue,[]);self.assertGreater(self.n.motion_poses()[2],self.n.now()-.1)
 def test_stale_candidate_defers_without_cancelling_active_action(self):
  self.goal();self.n.active=True;before=self.n.route.copy();self.n.sensor_received['/localization/odometry']-=1.
  msg=Path();msg.header.frame_id='map';msg.header.stamp=self.n.get_clock().now().to_msg()
  for a in before:
   p=PoseStamped();p.pose.position.x=float(a[0]);p.pose.orientation.w=1.;msg.poses.append(p)
  self.n.receive(msg)
  self.assertIsNone(self.n.recovery);np.testing.assert_allclose(self.n.route,before);self.assertIsNotNone(self.n.pending_audit_path);self.assertEqual(self.n.transport.calls,[])
 def native_case(self,gap_duration):
  self.goal();n=self.n;began=time.monotonic();events=[];outer=self
  class Handle:
   accepted=True;status=2
   def get_result_async(self):return Future()
   def cancel_goal_async(self):events.append(('cancel',time.monotonic()-began));self.status=5;return Future()
  handle=Handle()
  class Client:
   requests=0;callback=None
   def wait_for_server(self,**kw):return True
   def send_goal_async(self,goal,feedback_callback):
    self.requests+=1;self.callback=feedback_callback;v=Future();v.set_result(handle);return v
  client=Client();n.mppi_client=client
  n.destroy_timer(self.timer);gap=[False];observed=[]
  def feed():
   self.refresh();now=time.monotonic();elapsed=now-began
   gap[0]=.22<=elapsed<.22+gap_duration
   if gap[0]:
    m=copy.deepcopy(n.pose_snapshot);m.header.stamp=Time(nanoseconds=n.get_clock().now().nanoseconds-500000000).to_msg();n.receive_chassis_snapshot(m)
   st=n.get_clock().now().to_msg();sdk=TwistStamped();sdk.header.stamp=st;sdk.twist.linear.x=.08
   n.mppi_received.update(inputs=(now,Bool(data=True)),sdk=(now,sdk),status=(now,String(data='preview_ready')))
   if client.callback:client.callback(None)
   if elapsed>.85:
    observed.append(dict(requests=client.requests,cancelled=any(e[0]=='cancel' for e in events),target=n.target is not None,sent_after_gap=any(e[0]=='send' and e[1]>.22+gap_duration and e[2] for e in events)));n.seq+=1
  def send(command):
   events.append(('send',time.monotonic()-began,bool(any(command))));n.transport.calls.append(tuple(command))
   if any(command) and gap[0]:raise ValueError('unsafe_stale_send')
  n.transport.send=send;timer=n.create_timer(.015,feed);feed()
  try:
   with self.assertRaisesRegex(ValueError,'goal_changed' if gap_duration<.3 else 'input outage exceeded .30 seconds'):n.track_route()
  finally:n.destroy_timer(timer)
  if gap_duration<.3:
   self.assertTrue(observed);self.assertEqual(observed[0],dict(requests=1,cancelled=False,target=True,sent_after_gap=True))
  self.assertTrue(any(e[0]=='send' and not e[2] for e in events));return events
 def test_short_pose_gap_retains_native_controller_and_resumes_after_fresh_proof(self):self.native_case(.10)
 def test_long_pose_gap_still_exits_to_existing_recovery(self):self.native_case(.50)
if __name__=='__main__':unittest.main()
