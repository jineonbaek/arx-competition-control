"""A 2cm final via still receives correction at 4cm; no early map bypass."""
import time,unittest,rclpy
import test_retained_faults_ros as f
class TightTaskVia(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init());tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.RetainedFaults.setUp;tearDown=f.RetainedFaults.tearDown
 refresh=f.RetainedFaults.refresh;add=f.RetainedFaults.add;prepare=f.RetainedFaults.prepare
 def test_native_translation_keeps_correcting_inside_four_cm_for_two_cm_via(self):
  self.prepare('tight_via',worker=True)
  p=self.n.waypoints.route.points[0];p.update(type='via',pass_radius_m=.02)
  self.position[:]=[1.96,0,0];self.refresh()
  timer=self.n.create_timer(5.,lambda:self.n.cancel('fixture_cancel') if self.n.target is not None else None)
  try:
   with self.n.transport.task_scope(self.identity):self.assertEqual(self.n.run_goal(),'arrived_stopped')
  finally:self.n.destroy_timer(timer)
  self.assertFalse(self.paused)
  self.assertTrue(any(m['type']!='chassis.end_output' and any((m['x'],m['y'],m['omega'])) for m in self.sends))
  self.assertEqual(self.n.waypoints.route.index,1);self.assertEqual(self.n.waypoints.route.mode,'navigation')
