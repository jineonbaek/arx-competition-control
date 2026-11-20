"""Virtual time, real recovery loops, inert HTTP; no ROS node or hardware."""
import io,json,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import special_runtime as runtime

class VirtualDriver:
 def __init__(self,freeze=None,unready=None,moving=None):
  self.t=10.;self.seq=1;self.started=10.;self.freeze=freeze;self.unready=unready;self.moving=moving;self.statuses=[];self.validated=[]
  self.waypoints=SimpleNamespace(show=lambda *a:None)
  self.transport=SimpleNamespace(client=SimpleNamespace(open=self.console),origin='http://inert.invalid')
  self.sensor={}
 def source(self):
  elapsed=round(self.t-10.,6);elapsed=min(elapsed,self.freeze) if self.freeze is not None else elapsed
  return round(10.+math.floor((elapsed+1e-8)*10)*.1,6)
 def now(self):return self.t
 def current_intent(self,epoch):
  if self.t>=14.5 or epoch!=self.seq:raise ValueError('goal_changed')
 def soft_brake(self):return True
 def console(self,*a,**kw):
  age=self.t-10.;ready=not(self.unready and self.unready[0]<=age<self.unready[1])
  return io.BytesIO(json.dumps(dict(execution_enabled=True,nav_sdk_ready=True,body=dict(fresh=ready),capabilities=dict(chassis=''),chassis_active=False)).encode())
 def special_poses(self):
  source=self.source()
  if self.t-source>.65:raise ValueError('special_pose_stale')
  stamp=SimpleNamespace(sec=int(source),nanosec=round((source-int(source))*1e9))
  self.sensor={key:stamp for key in ('/pb/scan','/localization/odometry')}
  elapsed=source-10.;x=.02 if self.moving and self.moving[0]<=elapsed<self.moving[1] else 0.
  return np.array([x,0.,0.]),np.array([x,0.,0.]),source
 def validate_motion_proof(self,source):
  if source!=self.source() or self.t-source>.65:raise ValueError('motion_pose_stale_or_invalid')
  self.validated.append(source)
 def status(self,value):self.statuses.append(value)
 def spin(self,seconds):self.t=round(self.t+seconds,6)

class RecoveryCadence(unittest.TestCase):
 def run_wait(self,n):
  with patch.object(runtime,'time',SimpleNamespace(monotonic=lambda:n.t)),patch.object(runtime.rclpy,'ok',return_value=True):
   return runtime.wait_special_ready(n,1)
 def test_10hz_source_20hz_checks_complete_with_distinct_frame_proof(self):
  n=VirtualDriver();result=self.run_wait(n)
  self.assertGreaterEqual(n.t,11.35);self.assertLess(n.t,12.)
  self.assertEqual(len(n.validated),1);self.assertEqual(result[2],n.validated[0]);self.assertIn('special_inputs_restored',n.statuses)
 def test_repeated_frozen_source_cannot_fake_new_evidence(self):
  n=VirtualDriver(freeze=1.)
  with self.assertRaisesRegex(ValueError,'goal_changed'):self.run_wait(n)
  self.assertEqual(n.validated,[]);self.assertNotIn('special_inputs_restored',n.statuses)
 def test_new_frame_movement_revokes_cached_stationary_result(self):
  n=VirtualDriver(moving=(1.1,1.3));self.run_wait(n)
  self.assertGreaterEqual(n.t,12.55);self.assertEqual(len(n.validated),1)
 def test_changed_pose_with_same_stamp_invalidates_cached_proof(self):
  n=VirtualDriver();normal_source=n.source;normal_poses=n.special_poses
  n.source=lambda:11. if 1.<=n.t-10.<1.4 else normal_source()
  def changed():
   mp,od,source=normal_poses()
   if 1.1<=n.t-10.<1.3:mp[0]=od[0]=.02
   return mp,od,source
  n.special_poses=changed;self.run_wait(n);self.assertGreaterEqual(n.t,12.7)
 def test_pose_revision_on_window_completion_tick_cannot_reuse_old_verdict(self):
  n=VirtualDriver();normal_poses=n.special_poses
  def revised():
   mp,od,source=normal_poses()
   if n.t>=11.35:mp[0]=od[0]=.2
   return mp,od,source
  n.special_poses=revised;self.run_wait(n);self.assertGreaterEqual(n.t,12.7)
 def test_unready_input_resets_cached_proof_before_recovery(self):
  n=VirtualDriver(unready=(1.1,1.4));self.run_wait(n)
  self.assertGreaterEqual(n.t,12.75);self.assertEqual(len(n.validated),1)

if __name__=='__main__':unittest.main()
