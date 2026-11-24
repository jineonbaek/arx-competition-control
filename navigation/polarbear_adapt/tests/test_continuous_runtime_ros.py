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
 def test_last_precision_flows_to_timed_action_without_zero(self):
  self.start_special(.35,mid=.16);self.simulate=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped')
  calls=self.n.transport.calls
  first=next(i for i,c in enumerate(calls) if any(c))
  zero=next(i for i in range(first,len(calls)) if not any(calls[i]))
  self.assertTrue(all(not any(c) for c in calls[zero:]))
  self.assertGreater(self.position[0],.9)
  self.assertTrue(any(abs(c[0]-.225)<1e-8 for c in calls[first:zero]))
 def test_cancel_during_continuation_brakes_without_arrival(self):
  self.start_special(.12);self.simulate=True;self.cancel_after=22
  with self.assertRaises(ValueError):self.n.run_goal()
  self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.))
  self.assertFalse(self.n.waypoints.route.running)
 def test_special_before_task_does_not_stop_at_special(self):
  self.first_stop_x=None;self.saw_motion=False
  _,self.lease=self.worker();self.add('special',0.);self.add('special',.16);self.add('task',.4,'point_1')
  self.n.waypoints.start(None);self.n.seen=self.n.seq;self.simulate=True
  self.assertEqual(self.n.waypoints.route.mode,'task_direct')
  self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertGreater(self.position[0],.3)
  calls=self.n.transport.calls;first=next(i for i,c in enumerate(calls) if any(c));zero=next(i for i in range(first,len(calls)) if not any(calls[i]))
  self.assertGreater(self.first_stop_x,.3)

 def test_lost_hold_permission_brakes_during_timed_action(self):
  import io
  self.start_special(.12);self.simulate=True
  def state(*args,**kwargs):
   route=self.n.waypoints.route
   allowed=len(self.n.transport.calls)<24
   permit=dict(session_active=True,authorized=allowed,route_id=route.route_id,mission_id=route.mission_id,
               point_id=route.points[-1]['id'],sdk_forward=.3,duration_s=6.)
   return io.BytesIO(json.dumps(dict(execution_enabled=True,nav_sdk_ready=True,body=dict(fresh=True),
           capabilities=dict(chassis=''),terminal_forward=permit)).encode())
  self.n.transport.open=state
  from special_runtime import run_special
  with self.assertRaisesRegex(ValueError,'permit_lost'):run_special(self.n)
  self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.))
  self.assertNotEqual(self.n.waypoints.route.state,'completed')
