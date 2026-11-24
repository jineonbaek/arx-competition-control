"""ROS197 and an in-process fake transport only; no hardware output."""
import unittest,os,json,time
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import rclpy,numpy as np
import test_special_ros as fixtures
from terminal_forward import TerminalForward
class ThroughROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=fixtures.SpecialROS.setUp
 tearDown=fixtures.SpecialROS.tearDown
 def refresh(self):
  fixtures.SpecialROS.refresh(self)
  if hasattr(self,'first_stop_x') and self.first_stop_x is None and self.n.transport.calls:
   if any(self.n.transport.calls[-1]):self.saw_motion=True
   elif self.saw_motion:self.first_stop_x=float(self.position[0])
  if getattr(self,'lease',None):self.n.waypoints.hub.heartbeat(self.lease)
 add=fixtures.SpecialROS.add
 worker=fixtures.SpecialROS.worker
 event=fixtures.SpecialROS.event
 start_special=fixtures.SpecialROS.start_special
 def test_final_special_does_not_correct_saved_heading(self):
  self.add('special',0.);self.add('special',.3)
  self.n.waypoints.route.points[-1]['pose'][2]=.6
  self.n.waypoints.start(None);self.n.seen=self.n.seq;self.simulate=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertFalse(self.n.special_session['chain'].terminal_precision)
  self.assertTrue(all(abs(c[2])<1e-8 for c in self.n.transport.calls))
