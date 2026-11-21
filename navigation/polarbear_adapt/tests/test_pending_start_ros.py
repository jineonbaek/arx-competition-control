"""Actual ROS driver and route; domain197 and inert chassis only."""
import os,time,unittest,tempfile,json
from pathlib import Path
from unittest.mock import patch
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
import test_special_ros as f

class PendingStartROS(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 add=f.SpecialROS.add;worker=f.SpecialROS.worker
 def setUp(self):
  self.run=tempfile.TemporaryDirectory();self.env=patch.dict(os.environ,{'ARX_ADAPT_RUN':self.run.name,'ARX_DESKTOP_MANAGED':'0'})
  self.env.start();Path(self.run.name,'competition-map').touch();self.no_scan=False;self.no_pose=False;self.freeze=False
  f.SpecialROS.setUp(self);self.add('special',.18)
 def tearDown(self):f.SpecialROS.tearDown(self);self.env.stop();self.run.cleanup()
 def refresh(self):
  if self.freeze:return
  f.SpecialROS.refresh(self)
  if self.no_scan:self.n.sensor.pop('/pb/scan',None)
  if self.no_pose:
   m=PoseArray();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg();self.n.receive_chassis_snapshot(m)
 def spin(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:self.n.spin(.03)
 def wait_start(self):
  self.no_scan=True;self.refresh();self.n.waypoints.start(None)
  self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
  self.assertEqual(self.n.transport.calls,[])
 def test_scan_recovers_and_starts_once_after_continuous_updates(self):
  self.wait_start();self.no_scan=False;self.refresh();self.spin(.18)
  self.assertIsNone(self.n.target)
  self.spin(.65);self.assertIsNotNone(self.n.target);self.assertEqual(self.n.waypoints.route.mode,'direct')
  seq=self.n.seq;self.spin(.3);self.assertEqual(self.n.seq,seq);self.assertEqual(self.n.transport.calls,[])
 def test_pose_recovers_and_starts_without_second_click(self):
  self.no_pose=True;self.refresh();self.n.waypoints.start(None);self.assertIsNone(self.n.target)
  self.no_pose=False;self.refresh();self.spin(.9);self.assertIsNotNone(self.n.target)
 def test_one_fresh_batch_cannot_start(self):
  self.wait_start();self.no_scan=False;self.refresh();self.freeze=True;self.spin(.75)
  self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
 def test_cancelled_pending_request_never_revives(self):
  self.wait_start();self.n.cancel('cancelled');self.no_scan=False;self.refresh();self.spin(.9)
  self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
 def test_clear_cancels_pending_request(self):
  self.wait_start();self.n.waypoints.clear(None);self.no_scan=False;self.refresh();self.spin(.9)
  self.assertIsNone(self.n.target);self.assertEqual(self.n.waypoints.route.points,[])
 def test_route_edit_cancels_old_request(self):
  self.wait_start();self.add('special',.4);self.no_scan=False;self.refresh();self.spin(.9)
  self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
 def test_external_command_epoch_cancels_old_request(self):
  self.wait_start();self.external_epoch+=1;self.no_scan=False;self.refresh();self.spin(.9)
  self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
 def test_execution_disabled_cancels_even_after_reenabled(self):
  self.wait_start();old=self.n.waypoints.runtime.console
  self.n.waypoints.runtime.console=lambda:dict(execution_enabled=False,body=dict(fresh=True),command_epoch=1)
  self.spin(.2);self.n.waypoints.runtime.console=old;self.no_scan=False;self.refresh();self.spin(.9)
  self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
 def test_repeated_click_keeps_original_wait_and_starts_once(self):
  self.wait_start();self.no_scan=False;self.refresh()
  for _ in range(6):self.n.waypoints.start(None);self.spin(.12)
  self.assertIsNotNone(self.n.target);seq=self.n.seq;self.spin(.25);self.assertEqual(self.n.seq,seq)
 def test_execution_gate_reopening_requires_fresh_updates(self):
  gate=Path(self.run.name,'ready');self.n.ready_file=gate;self.n.waypoints.start(None)
  self.assertIsNone(self.n.target);gate.touch();self.spin(.2);self.assertIsNone(self.n.target)
  self.spin(.65);self.assertIsNotNone(self.n.target)
 def test_scan_gap_resets_continuous_recovery_window(self):
  self.wait_start();self.no_scan=False;self.refresh();self.spin(.2)
  self.no_scan=True;self.refresh();self.spin(.2);self.no_scan=False;self.refresh();self.spin(.2)
  self.assertIsNone(self.n.target);self.spin(.65);self.assertIsNotNone(self.n.target)
 def test_manual_override_revokes_wait_without_reviving(self):
  self.wait_start();self.n.waypoints.override();self.no_scan=False;self.refresh();self.spin(.9)
  self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
 def test_dispatch_input_race_restores_draft_then_retries(self):
  old=self.n.accept_special_goal;calls=[]
  def dispatch(m):
   calls.append(m)
   if len(calls)==1:raise ValueError('special_sensor_stale /pb/scan')
   return old(m)
  self.n.accept_special_goal=dispatch;before=self.n.waypoints.route.definition();self.n.waypoints.start(None)
  self.assertFalse(self.n.waypoints.route.running);self.assertEqual(self.n.waypoints.route.definition(),before)
  self.spin(.9);self.assertIsNotNone(self.n.target);self.assertEqual(len(calls),2)
 def test_unknown_control_epoch_does_not_arm_future_start(self):
  self.no_scan=True;self.refresh();old=self.n.waypoints.runtime.console
  self.n.waypoints.runtime.console=lambda:{k:v for k,v in old().items() if k!='command_epoch'}
  self.n.waypoints.start(None);self.n.waypoints.runtime.console=old
  self.no_scan=False;self.refresh();self.spin(.9);self.assertIsNone(self.n.target)
 def test_invalid_task_configuration_is_rejected_without_pending_replay(self):
  self.n.waypoints.clear(None);self.add('task',.2,'point_1');self.n.waypoints.start(None)
  self.worker();self.spin(.9);self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running)
if __name__=='__main__':unittest.main()
