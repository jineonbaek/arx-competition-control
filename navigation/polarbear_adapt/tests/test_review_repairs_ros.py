"""Four final-review regressions, ROS197 and inert actuator transport only."""
import copy,math,time,unittest
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np,rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool,String
from path_audit import AuditGrid
from async_path_audit import AuditPending
from mppi_trial.session_policy import changed_route
import test_mission_ros as fixtures
class SelectGrid(AuditGrid):
 pending=True
 def along(self,p):
  if self.pending and len(p)>2:raise AuditPending()
  return AuditGrid.along(self,p)
class ReviewRepairs(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init());tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=fixtures.MissionROS.setUp;tearDown=fixtures.MissionROS.tearDown;refresh=fixtures.MissionROS.refresh;add=fixtures.MissionROS.add
 def setup_goal(self,via=False):
  if via:self.add('via',.5)
  self.add('stop',2.);self.n.waypoints.start(None);self.n.seen=self.n.seq;self.n.request=self.n.now()-1.
  self.n.route=np.array([[0.,0.,0.],[.5,0.,0.],[2.,0.,0.]]);self.n.route_time=self.n.now();self.n.active=True
 def msg(self,age=0.,middle_y=0.):
  m=Path();m.header.frame_id='map';m.header.stamp=Time(nanoseconds=int((self.n.now()-age)*1e9)).to_msg()
  for x,y in ((0.,0.),(.5,middle_y),(2.,0.)):
   p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;m.poses.append(p)
  return m
 def test_old_pending_cannot_replace_newer_accepted_same_goal(self):
  self.setup_goal();g=self.n.grids['map'][1];g=SelectGrid(g.grid,g.origin,g.res,g.poly);self.n.grids['map']=(self.n.get_clock().now().to_msg(),g)
  a=self.msg(.10);b=self.msg(.05,.04);self.n.receive(a);g.pending=False;self.n.receive(b);accepted=self.n.route.copy();stamp=self.n.last_candidate_stamp
  self.n.retry_audit_path();self.n.receive(a);np.testing.assert_allclose(self.n.route,accepted);self.assertEqual(self.n.last_candidate_stamp,stamp)
 def test_missing_via_candidate_keeps_safely_proved_old_route(self):
  self.setup_goal(True);before=self.n.route.copy();m=self.msg(0.,.4);self.n.receive(m)
  self.assertIsNone(self.n.recovery);np.testing.assert_allclose(self.n.route,before);self.assertEqual(self.n.transport.calls,[])
 def test_wrong_frame_candidate_cannot_brake_or_replace_pending(self):
  self.setup_goal(True);before=self.n.route.copy();m=self.msg(0.,.4);m.header.frame_id='odom';self.n.receive(m)
  self.assertIsNone(self.n.recovery);np.testing.assert_allclose(self.n.route,before);self.assertEqual(self.n.transport.calls,[])
 def active_geometry_case(self,y):
  self.setup_goal();a=np.c_[np.linspace(0.,2.,9),np.zeros(9),np.zeros(9)];b=a.copy();b[1:-1,1]=y
  self.n.active_controller_route=a;self.n.route=b;self.position[1]=0.;self.refresh()
  point=[.65,-.36];g=self.n.grids['map'][1];col,row=np.floor((point-g.origin)/g.res).astype(int);g.grid[row,col]=100
  return a,b
 def test_pending_action_acceptance_checks_actual_controller_route(self):
  a,b=self.active_geometry_case(.3)
  with self.assertRaisesRegex(ValueError,'map_sweep_blocked'):self.n.validate_active_prefix(self.position,self.position)
 def test_suppressed_replacement_still_checks_actual_controller_route(self):
  a,b=self.active_geometry_case(.07);self.assertFalse(changed_route(a,b))
  with self.assertRaisesRegex(ValueError,'map_sweep_blocked'):self.n.validate_active_prefix(self.position,self.position)
 def test_source_crossing_400ms_during_sweep_never_sends_nonzero(self):
  self.setup_goal();self.n.active=False
  self.n.destroy_timer(self.timer) # Only the aged-source feed may refresh this fixture.
  def feed():
   self.refresh();m=copy.deepcopy(self.n.pose_snapshot);m.header.stamp=Time(nanoseconds=self.n.get_clock().now().nanoseconds-385000000).to_msg();self.n.receive_chassis_snapshot(m)
   now=time.monotonic();sdk=TwistStamped();sdk.header.stamp=self.n.get_clock().now().to_msg();sdk.twist.linear.x=.1
   self.n.mppi_received.update(inputs=(now,Bool(data=True)),sdk=(now,sdk),status=(now,String(data='preview_ready')))
   if client.callback:client.callback(None)
  class Client:
   callback=None
   def wait_for_server(self,**kw):return True
   def send_goal_async(self,goal,feedback_callback):
    self.callback=feedback_callback;h=SimpleNamespace(accepted=True,status=2,cancel_goal_async=lambda:None,get_result_async=lambda:Future());f=Future();f.set_result(h);return f
  client=Client();self.n.mppi_client=client;timer=self.n.create_timer(.01,feed);feed()
  def send(c):
   self.n.transport.calls.append(tuple(c))
   if any(c):raise ValueError('unguarded_nonzero_send')
  self.n.transport.send=send
  import mppi_goal_drive
  original=mppi_goal_drive.live_prefix
  def expensive(*args,**kw):
   result=original(*args,**kw);time.sleep(.025);return result
  with patch('mppi_goal_drive.live_prefix',expensive):
   with self.assertRaisesRegex(ValueError,'input outage exceeded .30 seconds'):self.n.track_route()
  self.assertTrue(all(not any(c) for c in self.n.transport.calls));self.n.destroy_timer(timer)
