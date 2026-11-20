"""ROS197 only: real navigation+handoff, inert physical output."""
import os,math,time,unittest,numpy as np,rclpy
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import test_special_ros as f
class T2ParkingROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.SpecialROS.setUp;tearDown=f.SpecialROS.tearDown;add=f.SpecialROS.add;worker=f.SpecialROS.worker
 def refresh(self):
  f.SpecialROS.refresh(self)
  if getattr(self,'token',None):self.hub.heartbeat(self.token)
 def start(self,position,yaw=0.):
  self.hub,self.token=self.worker();self.position=np.array(position,float);self.refresh();self.add('task',.2,'point_2');self.n.waypoints.route.points[0]['pose'][2]=yaw;self.n.waypoints.start(None);self.n.seen=self.n.seq
 def test_left_residual_is_corrected_before_task_notification(self):
  self.start([.19,.005,0.]);self.simulate=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped');self.n.cancel('arrived_stopped')
  self.assertIsNotNone(self.hub.active);p=self.hub.active['arrival_pose'];goal=self.hub.active['goal_pose'];left=(p[0]-goal[0])*(-math.sin(p[2]))+(p[1]-goal[1])*math.cos(p[2])
  self.assertLessEqual(left,0.);self.assertLessEqual(np.linalg.norm(np.array(p[:2])-goal[:2]),.02)
  self.assertTrue(any(c[1]<0 for c in self.n.transport.calls))
 def test_last_source_left_bias_defers_and_redispatches_same_t2(self):
  self.start([.2,.003,0.]);r=self.n.waypoints.route;identity=(r.route_id,r.mission_id,r.points[0]['id']);self.n.cancel('arrived_stopped')
  self.assertIsNone(self.hub.active)
  deadline=time.monotonic()+3.
  while self.n.target is None and time.monotonic()<deadline:self.hub.heartbeat(self.token);self.n.spin(.05)
  self.assertIsNotNone(self.n.target);self.assertEqual((r.route_id,r.mission_id,r.points[0]['id']),identity);self.assertEqual(r.state,'running');self.assertIsNone(self.hub.active)
  self.assertFalse(any(any(c) for c in self.n.transport.calls))
 def test_cancel_during_pending_left_alignment_does_not_restart(self):
  self.start([.2,.003,0.]);self.n.cancel('arrived_stopped');self.assertIsNone(self.hub.active)
  self.n.cancel('cancelled');self.n.spin(.25);self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running);self.assertIsNone(self.hub.active)
 def retry_until_dispatch(self,seconds=3.):
  deadline=time.monotonic()+seconds
  while self.n.target is None and time.monotonic()<deadline:self.n.spin(.05)
 def test_realign_failed_dispatch_preserves_pending_request_and_attempt_budget(self):
  self.start([.2,.003,0.]);rt=self.n.waypoints.runtime;point=self.n.waypoints.route.points[0];policy=rt.t2_policy(point)
  policy.check([.17,-.001,0.],True);policy.check([.17,-.001,0.],True)
  self.assertTrue(policy.fallback);self.n.cancel('arrived_stopped');original=self.n.waypoints.dispatch;attempts=[]
  def dispatch(p):
   attempts.append(1)
   if len(attempts)==1:raise OSError('inert temporary dispatch failure')
   return original(p)
  self.n.waypoints.dispatch=dispatch;self.retry_until_dispatch()
  self.assertIsNotNone(self.n.target);self.assertEqual(len(attempts),2);self.assertIs(rt.t2_policy(point),policy);self.assertEqual(policy.checks,2);self.assertTrue(policy.fallback)
  self.assertIsNone(self.hub.active);self.assertFalse(any(any(c) for c in self.n.transport.calls))
 def test_late_dispatch_observation_error_does_not_dispatch_twice(self):
  self.start([.2,.003,0.]);self.n.cancel('arrived_stopped');original=self.n.waypoints.dispatch;attempts=[]
  def dispatch(p):
   attempts.append(1);original(p);raise OSError('inert late observation failure')
  self.n.waypoints.dispatch=dispatch;self.retry_until_dispatch();self.n.spin(.2)
  self.assertIsNotNone(self.n.target);self.assertEqual(len(attempts),1);self.assertIsNone(self.n.waypoints.runtime.pending_notification);self.assertIsNone(self.hub.active)
 def test_two_failed_precise_rounds_use_legacy_tolerance(self):
  self.start([.17,-.001,0.],yaw=math.radians(2));events=[];old=self.n.status
  self.n.status=lambda value:(events.append(value),old(value))
  self.assertEqual(self.n.run_goal(),'arrived_stopped');self.assertEqual(events.count('task_verifying_stopped'),2)
  self.assertIn('arrived_stopped_t2_fallback task_direct',events)
  self.n.cancel('arrived_stopped');self.assertEqual(self.hub.active['point_id'],'point_2')

class T2StopBudgetROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 tearDown=f.SpecialROS.tearDown;add=f.SpecialROS.add;worker=f.SpecialROS.worker
 def setUp(self):
  self.stop_at=None;self.corrected_at=None;self.during_coast=None
  f.SpecialROS.setUp(self);self.hub,self.token=self.worker();old=self.n.status
  def status(value):
   old(value)
   if value=='task_verifying_stopped' and self.stop_at is None:self.stop_at=time.monotonic()
   if value=='task_terminal_correction':self.corrected_at=time.monotonic();self.n.cancel('cancelled')
  self.n.status=status
 def refresh(self):
  if self.stop_at is not None:self.position[0]=.2+.10*min(4.5,time.monotonic()-self.stop_at)
  f.SpecialROS.refresh(self)
  if getattr(self,'token',None):self.hub.heartbeat(self.token)
 def test_coasting_past_four_seconds_keeps_zero_until_stopped_and_preserves_budget(self):
  self.position=np.array([.2,-.005,0.]);self.refresh();self.add('task',.2,'point_2');self.n.waypoints.start(None);self.n.seen=self.n.seq
  def capture():
   session=getattr(self.n,'special_session',None)
   if session:self.during_coast=session['chain'].arrival_policy.checks
  timer=self.n.create_timer(4.2,capture)
  try:
   with self.assertRaisesRegex(ValueError,'goal_changed'):self.n.run_goal()
  finally:self.n.destroy_timer(timer)
  self.assertGreaterEqual(self.corrected_at-self.stop_at,5.25);self.assertEqual(self.during_coast,0)
  self.assertEqual(self.n.special_session['chain'].arrival_policy.checks,1)

if __name__=='__main__':unittest.main()
