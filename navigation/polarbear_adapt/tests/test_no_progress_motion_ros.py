import os,time,unittest
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import rclpy
import test_special_ros as f
from special_runtime import run_special
class NoProgressMotion(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.SpecialROS.setUp;tearDown=f.SpecialROS.tearDown
 refresh=f.SpecialROS.refresh;add=f.SpecialROS.add;start_special=f.SpecialROS.start_special
 def test_stationary_three_seconds_does_not_brake_or_restart_output(self):
  self.start_special(.5);self.simulate=False;events=[];old=self.n.status;began=time.monotonic();identity=self.n.transport.current
  def status(v):events.append(v);old(v)
  self.n.status=status
  timer=self.n.create_timer(.04,lambda:self.n.cancel('cancelled') if time.monotonic()-began>4.1 and self.n.target is not None else None)
  try:
   with self.assertRaisesRegex(ValueError,'goal_changed'):run_special(self.n)
  finally:self.n.destroy_timer(timer)
  self.assertTrue(any(v.startswith('special_start_assist_sdk ') for v in events))
  self.assertNotIn('special_waiting_for_inputs',events)
  first=next(i for i,v in enumerate(self.n.transport.calls) if any(v));before_cancel=self.n.transport.calls[first:-1]
  self.assertTrue(before_cancel);self.assertTrue(all(any(v) for v in before_cancel));self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.));self.assertIsNone(self.n.target)
 def test_invalid_sensor_still_brakes_and_requires_fresh_input(self):
  self.start_special(.5);events=[];old=self.n.status;began=time.monotonic()
  def status(v):events.append(v);old(v)
  self.n.status=status
  def fault():
   if time.monotonic()-began>.25:self.n.sensor={}
   if time.monotonic()-began>.8 and self.n.target is not None:self.n.cancel('cancelled')
  timer=self.n.create_timer(.02,fault)
  try:
   with self.assertRaisesRegex(ValueError,'goal_changed'):run_special(self.n)
  finally:self.n.destroy_timer(timer)
  self.assertIn('special_waiting_for_inputs',events);self.assertTrue(any(not any(v) for v in self.n.transport.calls));self.assertIsNone(self.n.target)
if __name__=='__main__':unittest.main()
