"""Notification retry must retain stopped state, never multiply source evidence."""
import math,types,unittest
from unittest.mock import patch
import test_task_resume_retry as f
class NotificationCadence(unittest.TestCase):
 setUp=f.ResumeRetry.setUp;tearDown=f.ResumeRetry.tearDown
 def prepare(self):
  self.d.advance(2.1);self.r.notify(self.route.points[0]);self.r.hub.register(self.worker)
  self.source=round(self.d.t,1);self.freeze=False
  def poses():
   if self.d.t-self.source>.65:raise ValueError('motion_pose_stale_or_invalid')
   return self.d.position.copy(),self.d.position.copy(),self.source
  self.d.motion_poses=poses;self.d.special_poses=poses
  def validate(stamp):
   if stamp!=self.source or self.d.t-stamp>.65:raise ValueError('motion_pose_stale_or_invalid')
  self.d.validate_motion_proof=validate
 def tick(self,count):
  for _ in range(count):
   self.d.advance(.05)
   if not self.freeze:self.source=round(math.floor((self.d.t+1e-8)*10)/10,1)
   self.d.pose_snapshot.header.stamp=types.SimpleNamespace(sec=int(self.source),nanosec=round((self.source%1)*1e9))
   self.r.hub.heartbeat(self.token)
   with patch.object(f.runtime_module.time,'monotonic',lambda:self.d.t):self.r.tick()
 def test_pending_notification_10hz_source_20hz_tick_reaches_one_visit(self):
  self.prepare();self.tick(65)
  self.assertIsNotNone(self.r.hub.active);self.assertEqual(self.r.hub.active['status'],'notified')
  self.assertEqual(sum(e['event']=='notified' for e in self.r.hub.events),1);self.assertEqual(self.w.dispatched,[])
 def test_same_stamp_pose_revision_requires_new_stopped_window(self):
  self.prepare();self.tick(21);self.freeze=True;self.d.position[0]=.02;self.tick(2)
  self.d.position[0]=0.;self.tick(2);self.freeze=False;self.tick(8)
  self.assertIsNone(self.r.hub.active);self.tick(40)
  self.assertEqual(self.r.hub.active['status'],'notified')
 def test_freeze_does_not_complete_proof_then_new_sources_can_recover(self):
  self.prepare();self.tick(21);self.freeze=True;self.tick(20)  # Freeze after the first stable source, before a second revision.
  self.assertIsNone(self.r.hub.active)
  self.freeze=False;self.tick(65)
  self.assertEqual(self.r.hub.active['status'],'notified');self.assertEqual(self.w.dispatched,[])
if __name__=='__main__':unittest.main()
