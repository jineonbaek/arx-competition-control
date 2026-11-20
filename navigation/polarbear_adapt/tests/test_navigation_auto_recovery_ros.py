"""Long parked action outage spends no motion budget, private ROS only."""
import os,time,unittest
assert os.environ.get('ROS_DOMAIN_ID')=='197'
from types import SimpleNamespace
from unittest.mock import patch
import rclpy
import test_retained_faults_ros as f
class TrackingReady(Exception):pass
class NavigationAutoRecovery(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.RetainedFaults.setUp;tearDown=f.RetainedFaults.tearDown
 refresh=f.RetainedFaults.refresh;add=f.RetainedFaults.add;prepare=f.RetainedFaults.prepare
 def test_600_seconds_action_offline_does_not_exhaust_parked_motion_budget(self):
  self.prepare('mppi_unavailable');clock=time.monotonic;offset=[0.];ready=[False];started=self.n.started;intent=self.n.current_intent
  def unavailable_once(**kw):
   if not offset[0]:offset[0]=600.;return False
   ready[0]=True;return True
  def current(epoch):
   if ready[0]:raise TrackingReady()
   return intent(epoch)
  self.client.wait_for_server=unavailable_once;self.n.current_intent=current
  fake=SimpleNamespace(monotonic=lambda:clock()+offset[0])
  with patch('mppi_goal_drive.time',fake),patch('recovery_budget.time',fake),patch('retained_goal.time',fake):
   with self.assertRaises(TrackingReady):self.n.track_route()
   self.assertLess(fake.monotonic()-self.n.started,.5)
  self.assertGreaterEqual(self.n.started-started,600.);self.assertFalse(self.paused)
  self.assertEqual(self.n.transport.current,self.identity);self.assertTrue(all(not any(c) for c in self.n.transport.calls))
 def test_terminal_review_automatically_rechecks_without_new_task_or_output(self):
  self.prepare('terminal_recheck');self.n.terminal_checks=3;identity=self.n.transport.current
  timer=self.n.create_timer(3.,lambda:self.n.cancel('fixture_deadline'))
  try:self.n.pause_terminal_correction(self.n.seen,'outside_arrival_tolerance')
  finally:self.n.destroy_timer(timer)
  self.assertEqual(self.n.terminal_checks,0);self.assertEqual(self.n.transport.current,identity)
  self.assertTrue(self.n.waypoints.route.running);self.assertTrue(all(not any(c) for c in self.n.transport.calls))
if __name__=='__main__':unittest.main()
