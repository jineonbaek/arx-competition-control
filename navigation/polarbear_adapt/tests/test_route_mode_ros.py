"""Virtual body, inert transport, private ROS197 only."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import sys,json,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_special_ros as f
from std_msgs.msg import String
import rclpy
class TestRouteROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.SpecialROS.setUp;tearDown=f.SpecialROS.tearDown;add=f.SpecialROS.add
 def refresh(self):
  f.SpecialROS.refresh(self)
  if getattr(self,'parked',False) and self.n.target is not None:self.n.cancel('fixture_cancel')
 def start(self,which):
  self.statuses=[];self.parked=False;old=self.n.status
  def status(v):
   self.statuses.append(v);old(v)
   if v.startswith('paused_navigation'):self.parked=True
  self.n.status=status;self.add('task',.25,which);self.add('special',.8)
  self.n.waypoints.edit(String(data=json.dumps(dict(action='test_mode',enabled=True))))
  self.n.waypoints.start(None);self.n.seen=self.n.seq;self.simulate=True
 def test_t1_one_reverse_no_jobs_then_next_straight(self):
  self.blocked=True;self.refresh();self.start('point_1')
  self.assertEqual(self.n.run_goal(),'arrived_stopped');self.assertFalse(self.parked)
  reverse=[v for v in self.n.transport.calls if v[0]<0];self.assertTrue(reverse);self.assertTrue(all(v[1:]==(0.,0.) for v in reverse));self.assertEqual(self.statuses.count('test_t1_backoff'),1)
  done=[s for s in self.statuses if s.startswith('test_t1_backoff_complete ')];self.assertEqual(len(done),1);distance=float(done[0].split()[-1]);self.assertGreaterEqual(distance,.10);self.assertLessEqual(distance,.20)
  actual=self.position.copy();self.n.cancel('arrived_stopped');self.n.seen=self.n.seq
  self.assertEqual(self.n.waypoints.route.mode,'direct');self.assertAlmostEqual(self.n.waypoints.route.direct_source['pose'][0],actual[0]);self.assertIsNone(self.n.waypoints.hub.active)
  self.assertEqual(self.n.run_goal(),'arrived_stopped');self.assertGreater(max(v[0] for v in self.n.transport.calls),.225)
 def test_t2_never_reverses_and_next_segment_stays_direct(self):
  self.missing=True;self.refresh();self.start('point_2')
  self.assertEqual(self.n.run_goal(),'arrived_stopped');self.assertFalse(any(v[0]<0 for v in self.n.transport.calls));self.assertFalse(any(s.startswith('test_t1_backoff') for s in self.statuses));self.assertIsNone(self.n.waypoints.hub.active)
  self.assertLessEqual(max(v[0] for v in self.n.transport.calls),.225+1e-9)
  actual=self.position.copy();self.n.cancel('arrived_stopped');self.n.seen=self.n.seq;self.assertEqual(self.n.waypoints.route.mode,'direct');self.assertAlmostEqual(self.n.waypoints.route.direct_source['pose'][0],actual[0]);self.assertEqual(self.n.run_goal(),'arrived_stopped')
if __name__=='__main__':unittest.main()
