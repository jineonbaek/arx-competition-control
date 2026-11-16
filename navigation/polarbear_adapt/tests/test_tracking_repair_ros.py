"""Private ROS197 and inert transport: no motion or production endpoints."""
import copy,math,time,unittest
import numpy as np,rclpy
from geometry_msgs.msg import PoseArray,PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from concurrent.futures import Future
from types import SimpleNamespace
from rclpy.time import Time
import test_mission_ros as f
class RepairROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init());tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.MissionROS.setUp;tearDown=f.MissionROS.tearDown;refresh=f.MissionROS.refresh;add=f.MissionROS.add
 def goal(self):
  self.add('stop',2.);self.n.waypoints.start(None);self.n.seen=self.n.seq
 def test_fresh_authority_is_used_when_local_tf_is_missing(self):
  self.goal();self.n.tf.clear();np.testing.assert_allclose(self.n.checked(),self.position);mp,op,source=self.n.motion_poses();np.testing.assert_allclose(mp,op);self.assertGreater(source,0)
 def test_motion_source_400ms_bound_not_body_650ms(self):
  self.goal();m=copy.deepcopy(self.n.pose_snapshot);m.header.stamp=Time(nanoseconds=self.n.get_clock().now().nanoseconds-500000000).to_msg();self.n.receive_chassis_snapshot(m)
  with self.assertRaisesRegex(ValueError,'motion_pose'):self.n.checked()
 def test_healthy_390ms_motion_source_still_valid(self):
  self.goal();m=copy.deepcopy(self.n.pose_snapshot);m.header.stamp=Time(nanoseconds=self.n.get_clock().now().nanoseconds-390000000).to_msg();self.n.receive_chassis_snapshot(m);self.n.checked()
 def test_future_source_is_rejected(self):
  self.goal();m=copy.deepcopy(self.n.pose_snapshot);m.header.stamp=Time(nanoseconds=self.n.get_clock().now().nanoseconds+100000000).to_msg();self.n.receive_chassis_snapshot(m)
  with self.assertRaisesRegex(ValueError,'motion_pose'):self.n.checked()
 def test_withdrawn_authority_cannot_fallback_to_fresh_tf(self):
  self.goal();m=PoseArray();m.header.frame_id='map';self.n.receive_chassis_snapshot(m)
  with self.assertRaisesRegex(ValueError,'motion_pose'):self.n.checked()
 def test_relocalization_cannot_revive_delayed_snapshot(self):
  self.goal();old=copy.deepcopy(self.n.pose_snapshot);m=PoseWithCovarianceStamped();m.header.frame_id='map';self.n.relocalizing(m);self.n.receive_chassis_snapshot(old)
  with self.assertRaisesRegex(ValueError,'motion_pose'):self.n.checked()
 def test_via_controller_request_ends_at_next_unpassed_point(self):
  self.add('via',.4);self.add('stop',2.);self.n.waypoints.start(None);self.n.seen=self.n.seq
  route=np.array([[0.,0.,0.],[.4,0.,0.],[2.,0.,0.]])
  request=self.n.make_request(route);self.assertAlmostEqual(request.path.poses[-1].pose.position.x,.4);self.assertAlmostEqual(self.n.target[0],2.)
  self.position[0]=.35;self.refresh();self.n.waypoints.observe(self.position,self.position,self.n.now());request=self.n.make_request(route)
  self.assertAlmostEqual(request.path.poses[-1].pose.position.x,2.);self.assertEqual(self.n.waypoints.route.index,1)

class ImmediateHandle:
 accepted=True;status=4
 def get_result_async(self):
  f=Future();f.set_result(SimpleNamespace(status=4,result=SimpleNamespace(error_code=0,error_msg='')));return f
 def cancel_goal_async(self):return None
class ViaAction(RepairROS):
 def test_native_success_before_observed_via_cannot_finish_mission(self):
  self.add('via',.15);self.add('stop',.8);self.n.waypoints.start(None);self.n.seen=self.n.seq
  self.n.route=np.array([[0.,0.,0.],[.15,0.,0.],[.8,0.,0.]]);self.n.route_time=self.n.now()
  self.n.mppi_received['inputs']=(time.monotonic(),Bool(data=True))
  class Client:
   def wait_for_server(self,**kw):return True
   def send_goal_async(self,*a,**kw):
    f=Future();f.set_result(ImmediateHandle());return f
  self.n.mppi_client=Client()
  with self.assertRaisesRegex(ValueError,'via_not_observed'):self.n.track_route()
  self.assertEqual(self.n.waypoints.route.index,0);self.assertIsNotNone(self.n.target);self.assertTrue(self.n.waypoints.route.running)
 def test_observed_via_continues_to_next_target_without_stopped_verification(self):
  self.add('via',.15);self.add('stop',.8);self.n.waypoints.start(None);self.n.seen=self.n.seq
  self.n.route=np.array([[0.,0.,0.],[.15,0.,0.],[.8,0.,0.]]);self.n.route_time=self.n.now()
  self.n.mppi_received['inputs']=(time.monotonic(),Bool(data=True));outer=self;requests=[]
  class Client:
   def wait_for_server(self,**kw):return True
   def send_goal_async(self,goal,**kw):
    requests.append(goal.path.poses[-1].pose.position.x)
    if len(requests)==2:raise ValueError('fixture_second_request')
    outer.position[0]=.06;outer.refresh();f=Future();f.set_result(ImmediateHandle());return f
  self.n.mppi_client=Client()
  with self.assertRaisesRegex(ValueError,'fixture_second_request'):self.n.track_route()
  np.testing.assert_allclose(requests,[.15,.8]);self.assertEqual(self.n.waypoints.route.index,1);self.assertAlmostEqual(self.n.target[0],.8);self.assertEqual(self.n.transport.calls,[])
