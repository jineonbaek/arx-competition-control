"""Inert submissions prove retained retreat origin, progress and stopping latch."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from agent.chassis_retreat import ChassisRetreatStep

class Reader:
 def __init__(self,t):self.t=t;self.odom=np.array([2.,3.,np.pi/2]);self.error=None;self.age=.05;self.closed=False
 def now(self):return self.t[0]
 def read(self):
  if self.error:raise self.error
  return self.odom.copy(),self.odom.copy(),self.t[0]-self.age
 def check_motion(self,*args):pass
 def close(self):self.closed=True
class RetreatPoseWait(unittest.TestCase):
 def setUp(self):
  self.t=[50.];self.calls=[];self.reject_end=False;self.reader=Reader(self.t)
  def send(m,token):
   self.calls.append(dict(m));return {'ok':not(self.reject_end and m['type']=='chassis.end_output')}
  self.ctrl=NS(clock=lambda:self.t[0],submit=send);self.step=ChassisRetreatStep(self.ctrl,'owned')
  with patch('agent.chassis_retreat._PoseReader',return_value=self.reader):self.step.start()
 def poll(self,dt=.1):self.t[0]+=dt;return self.step.poll()
 def moves(self):return [m for m in self.calls if m['type']!='chassis.end_output' and any(m[k] for k in ('x','y','omega'))]
 def wait_valid(self):
  self.reader.error=None
  for _ in range(18):self.poll()
 def test_valid_delayed_pose_does_not_zero_or_interrupt_output(self):
  self.reader.age=.8
  for _ in range(5):self.poll()
  self.assertTrue(self.moves());self.assertFalse(any(m['type']=='chassis.end_output' for m in self.calls));self.assertIsNone(self.step.stop)
 def test_invalid_pose_waits_without_ending_program_then_resumes_original_target(self):
  self.poll();goal=self.step.target.copy();origin=self.step.route[0].copy();old=self.step.transport.current
  self.reader.odom[1]=2.96;self.reader.error=ValueError('motion_pose_stale_or_invalid')
  for _ in range(5):self.assertEqual(self.poll()['status'],'backing_chassis')
  self.assertEqual(self.calls[-1]['type'],'chassis.end_output');before=len(self.moves())
  self.wait_valid();self.assertGreater(len(self.moves()),before);self.assertGreater(self.step.transport.current.generation,old.generation)
  np.testing.assert_array_equal(self.step.target,goal);np.testing.assert_array_equal(self.step.route[0],origin)
 def test_source_change_at_send_boundary_is_waited_not_replayed_from_new_origin(self):
  self.poll();goal=self.step.target.copy();self.reader.error=ValueError('motion_pose_changed_during_proof')
  self.assertEqual(self.poll()['status'],'backing_chassis');self.wait_valid();np.testing.assert_array_equal(self.step.target,goal)
 def test_invalid_first_pose_never_moves_and_does_not_fail_step(self):
  self.reader.error=ValueError('motion_pose_stale_or_invalid');self.assertEqual(self.poll()['status'],'backing_chassis');self.assertEqual(self.moves(),[])
  self.wait_valid();self.assertTrue(self.moves())
 def test_cancel_while_waiting_never_restarts_reader_or_motion(self):
  self.poll();self.reader.error=ValueError('motion_pose_stale_or_invalid');self.poll();before=len(self.moves());self.step.cancel()
  self.assertEqual(len(self.moves()),before);self.assertTrue(self.reader.closed)
 def test_end_ack_failure_keeps_waiting_without_new_movement(self):
  self.poll();old=self.step.transport.current;self.reject_end=True;self.reader.error=ValueError('motion_pose_stale_or_invalid')
  self.assertEqual(self.poll()['status'],'backing_chassis');self.reader.error=None
  for _ in range(15):self.poll()
  self.assertEqual(self.step.transport.current,old);self.assertEqual(len(self.moves()),1)
  self.reject_end=False;self.wait_valid();self.assertGreater(self.step.transport.current.generation,old.generation)
 def test_stopping_latch_survives_invalid_pose_without_another_reverse(self):
  self.poll();self.reader.odom[1]=2.79;self.poll();before=len(self.moves());self.reader.error=ValueError('motion_pose_stale_or_invalid');self.assertEqual(self.poll()['status'],'backing_chassis')
  self.wait_valid();self.assertTrue(self.step.done);self.assertEqual(len(self.moves()),before)
 def test_invalid_output_authorization_is_not_hidden_as_pose_wait(self):
  self.poll();self.reader.error=ValueError('motion_pose_stale_or_invalid')
  self.ctrl.submit=lambda *a:(_ for _ in ()).throw(ValueError('motion authorization invalid'))
  with self.assertRaisesRegex(ValueError,'motion authorization invalid'):self.poll()
 def test_odometry_source_or_receipt_expiry_waits_and_resumes(self):
  self.poll();goal=self.step.target.copy();self.reader.error=ValueError("sensor_or_costmap_stale /localization/odometry")
  self.assertTrue(self.poll().get("waiting_for_pose"));self.wait_valid();np.testing.assert_array_equal(self.step.target,goal)
 def test_obstacle_rejection_is_not_hidden_as_pose_delay(self):
  self.reader.error=ValueError('map_footprint_blocked')
  with self.assertRaisesRegex(ValueError,'map_footprint_blocked'):self.poll()
if __name__=='__main__':unittest.main()
