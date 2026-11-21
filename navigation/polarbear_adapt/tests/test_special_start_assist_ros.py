import os,time,unittest
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import rclpy
import test_special_ros as f
from special_runtime import run_special
class StartAssistROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.SpecialROS.setUp;tearDown=f.SpecialROS.tearDown
 refresh=f.SpecialROS.refresh;add=f.SpecialROS.add;start_special=f.SpecialROS.start_special
 def attempt(self,test=False):
  self.start_special(.2)
  if test:
   self.n.waypoints.route.points[-1].update(type='special',test_task_id='point_1')
  self.simulate=False;events=[];old=self.n.status;began=time.monotonic()
  def status(v):events.append(v);old(v)
  self.n.status=status
  timer=self.n.create_timer(.04,lambda:self.n.cancel('cancelled') if time.monotonic()-began>7.1 and self.n.target is not None else None)
  try:
   with self.assertRaisesRegex(ValueError,'goal_changed'):run_special(self.n)
  finally:self.n.destroy_timer(timer)
  return events
 def test_stationary_normal_special_escalates_without_braking(self):
  events=self.attempt();self.assertIn('special_start_assist_sdk 0.30',events);self.assertIn('special_start_assist_sdk 0.60',events)
  first=next(i for i,c in enumerate(self.n.transport.calls) if any(c));self.assertTrue(all(any(c) for c in self.n.transport.calls[first:-1]));self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.));self.assertGreaterEqual(max(abs(c[0]) for c in self.n.transport.calls),.599)
 def test_test_task_approach_never_receives_start_boost(self):
  events=self.attempt(True);self.assertFalse(any(e.startswith('special_start_assist_sdk') for e in events));self.assertLessEqual(max(abs(c[0]) for c in self.n.transport.calls),.2250001)
if __name__=='__main__':unittest.main()
