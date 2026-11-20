"""A data gap must not restart a test-T1 reverse from a new origin."""
import os,time,unittest
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import rclpy
import test_special_ros as f
import test_route_mode_ros as mode
class BackoffRecovery(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.SpecialROS.setUp;tearDown=f.SpecialROS.tearDown
 add=f.SpecialROS.add;start=mode.TestRouteROS.start
 def refresh(self):
  f.SpecialROS.refresh(self)
  if getattr(self,'backoff_origin',None) is not None and not self.injected:
   traveled=self.backoff_origin[0]-self.position[0]
   if self.inject_at=='moving' and traveled>.035:
    self.injected=True;self.gap_until=time.monotonic()+.45
  if time.monotonic()<getattr(self,'gap_until',0.):self.n.sensor={}
  if getattr(self,'parked',False) and self.n.target is not None:self.n.cancel('fixture_cancel')
 def execute(self,when):
  self.inject_at=when;self.injected=False;self.backoff_origin=None;self.gap_until=0.;self.start('point_1');old=self.n.status
  def status(v):
   old(v)
   if v=='test_t1_backoff':self.backoff_origin=self.position.copy()
   if when=='stopped' and v=='test_t1_backoff_verifying_stopped' and not self.injected:
    self.injected=True;self.gap_until=time.monotonic()+.45;self.n.sensor={};self.reverse_at_stop=sum(c[0]<0 for c in self.n.transport.calls)
  self.n.status=status
  self.assertEqual(self.n.run_goal(),'arrived_stopped');self.assertTrue(self.injected);self.assertFalse(self.parked)
  distance=self.backoff_origin[0]-self.position[0];self.assertGreaterEqual(distance,.10);self.assertLessEqual(distance,.20)
  self.assertEqual(self.statuses.count('test_t1_backoff'),1);self.assertIsNone(self.n.waypoints.hub.active)
 def test_mid_reverse_gap_preserves_original_distance(self):self.execute('moving')
 def test_gap_after_target_latched_never_sends_another_reverse(self):
  self.execute('stopped');self.assertEqual(sum(c[0]<0 for c in self.n.transport.calls),self.reverse_at_stop)
if __name__=='__main__':unittest.main()
