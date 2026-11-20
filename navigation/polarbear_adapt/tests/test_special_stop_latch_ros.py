"""ROS197, inert SDK: prove coast cannot immediately re-enable reverse chasing."""
import os,time,unittest
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import rclpy
import test_special_ros as f
class StopLatch(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 tearDown=f.SpecialROS.tearDown;add=f.SpecialROS.add;start_special=f.SpecialROS.start_special
 def setUp(self):
  self.stopped_at=None;self.corrected_at=None;self.start_motion=None;self.stop_position=None
  f.SpecialROS.setUp(self);old=self.n.status
  def status(v):
   old(v)
   if v=='special_verifying_stopped' and self.stopped_at is None:self.stopped_at=time.monotonic();self.stop_position=float(self.position[0])
   if v=='special_terminal_correction':self.corrected_at=time.monotonic();self.n.cancel('cancelled')
  self.n.status=status
 def refresh(self):
  now=time.monotonic()
  if self.stopped_at is not None:self.position[0]=self.stop_position+.4*min(.5,now-self.stopped_at)
  elif self.n.transport.calls and any(self.n.transport.calls[-1]):
   if self.start_motion is None:self.start_motion=now
   self.position[0]=.4*(now-self.start_motion)
  f.SpecialROS.refresh(self)
 def test_outside_radius_coast_stays_zero_until_new_stationary_proof(self):
  self.start_special(.2)
  with self.assertRaisesRegex(ValueError,'goal_changed'):self.n.run_goal()
  self.assertIsNotNone(self.corrected_at);self.assertGreaterEqual(self.corrected_at-self.stopped_at,1.3)
  self.assertGreaterEqual(sum(not any(c) for c in self.n.transport.calls),8)

class OutputStopRecovery(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 tearDown=f.SpecialROS.tearDown;add=f.SpecialROS.add;start_special=f.SpecialROS.start_special
 def setUp(self):
  self.fault_at=None;self.fault_x=None
  f.SpecialROS.setUp(self)
 def refresh(self):
  if self.fault_at is not None:
   self.simulate=False;self.position[0]=self.fault_x+.6*min(.6,time.monotonic()-self.fault_at)
  f.SpecialROS.refresh(self)
 def test_first_terminal_zero_failure_retains_low_speed_after_outer_recovery(self):
  import numpy as np
  self.start_special(.5);self.simulate=True;old=self.n.transport.current;submit=self.n.transport.submit;moving_seen=[False];resumed=[]
  def send(m):
   moving=m['type']!='chassis.end_output' and any(m[k] for k in ('x','y','omega'))
   old_generation=m['generation']==old.generation
   if old_generation and moving:moving_seen[0]=True
   if m['type']!='chassis.end_output' and old_generation and self.fault_at is not None:raise ValueError('底盘任务已结束或指令序号已过期')
   if m['type']!='chassis.end_output' and not moving and moving_seen[0] and self.fault_at is None:
    self.fault_at=time.monotonic();self.fault_x=float(self.position[0]);raise ValueError('底盘指令已过期或早于维护结束')
   if moving and not old_generation:
    resumed.append(float(np.hypot(m['x'],m['y'])))
    if len(resumed)>=3:self.n.cancel('cancelled')
   return submit(m)
  self.n.transport.submit=send
  with self.assertRaisesRegex(ValueError,'goal_changed'):
   with self.n.transport.task_scope(old):self.n.run_goal()
  self.assertEqual(len(resumed),3);self.assertLessEqual(max(resumed),.14+1e-9)

if __name__=='__main__':unittest.main()
