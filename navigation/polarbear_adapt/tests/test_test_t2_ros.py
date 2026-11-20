"""Private ROS197, inert transport: test T2 shares formal parking, never jobs."""
import os,math,unittest,json,numpy as np,rclpy
assert os.environ.get('ROS_DOMAIN_ID')=='197'
import test_route_mode_ros as route_fixture
from std_msgs.msg import String
from t2_arrival import left_error
class TestT2PrecisionROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=route_fixture.TestRouteROS.setUp;tearDown=route_fixture.TestRouteROS.tearDown;add=route_fixture.TestRouteROS.add;refresh=route_fixture.TestRouteROS.refresh
 def begin(self,position):
  self.position=np.array(position,float);self.refresh();self.statuses=[];self.parked=False;old=self.n.status
  def status(v):self.statuses.append(v);old(v)
  self.n.status=status;self.add('task',.25,'point_2');self.add('special',.8)
  self.n.waypoints.edit(String(data=json.dumps(dict(action='test_mode',enabled=True))))
  self.n.waypoints.start(None);self.n.seen=self.n.seq
 def test_test_t2_corrects_left_and_yaw_without_worker_or_fixed_backoff(self):
  self.begin([.24,.006,math.radians(3)]);self.simulate=True;self.missing=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped')
  goal=np.array([.25,0.,0.]);self.assertLessEqual(np.linalg.norm(self.position[:2]-goal[:2]),.02)
  self.assertLessEqual(abs(self.position[2]),math.radians(1));self.assertLessEqual(left_error(self.position,goal),0.)
  self.assertTrue(any(v[1]<0 for v in self.n.transport.calls));self.assertIsNone(self.n.waypoints.hub.active)
  self.assertFalse(any(s.startswith('test_t1_backoff') for s in self.statuses))
  session=self.n.special_session;policy=session['chain'].arrival_policy
  self.assertIs(policy,session['review'].arrival_policy);self.assertIs(policy,self.n.waypoints.runtime.t2_policy(session['control_point']))
  self.assertEqual(self.n.waypoints.route.points[0]['type'],'special')
  actual=self.position.copy();self.n.cancel('arrived_stopped');self.n.seen=self.n.seq
  self.assertEqual(self.n.waypoints.route.mode,'direct');np.testing.assert_allclose(self.n.waypoints.route.direct_source['pose'],actual)
  self.assertEqual(self.n.run_goal(),'arrived_stopped');self.assertGreater(max(v[0] for v in self.n.transport.calls),.225)
  self.assertIsNone(self.n.waypoints.hub.active)
 def test_test_t2_two_stopped_failures_fallback_without_job(self):
  self.begin([.22,-.003,math.radians(2)])
  self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertEqual(self.statuses.count('task_verifying_stopped'),2)
  self.assertIn('arrived_stopped_t2_fallback test_task_direct',self.statuses)
  self.assertEqual(self.n.special_session['chain'].arrival_policy.checks,2)
  self.assertIsNone(self.n.waypoints.hub.active)
if __name__=='__main__':unittest.main()
