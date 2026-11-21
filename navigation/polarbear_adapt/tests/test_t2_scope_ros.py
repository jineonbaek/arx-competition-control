import os,json,unittest,rclpy,numpy as np
assert os.environ.get('ROS_DOMAIN_ID')=='197'
from std_msgs.msg import String
import test_special_ros as fixture
class ScopeROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init());tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=fixture.SpecialROS.setUp;tearDown=fixture.SpecialROS.tearDown;refresh=fixture.SpecialROS.refresh;add=fixture.SpecialROS.add
 def prepare(self):
  self.position=np.array([5.,0.,0.]);self.refresh()
  for i in range(1,7):self.add('task' if i in (2,6) else 'special',float(i),'point_1' if i==2 else 'point_2' if i==6 else None)
  self.n.waypoints.hub.register(dict(worker_id='battery',instance_id='10000000-0000-4000-8000-000000000001',point_ids=['point_2']))
 def test_callback_starts_current_point_five_with_six_point_definition_retained(self):
  self.assertTrue(callable(getattr(self.n.waypoints,'start_t2',None)),'T2 callback missing after Git rollback')
  self.prepare();before=self.n.waypoints.route.definition();self.n.waypoints.start_t2(String(data=json.dumps(dict(action='start',request_id='run-start',route_id=before['route_id'],expected_definition=before))))
  self.assertIsNotNone(self.n.target);self.assertEqual(self.n.waypoints.route.index,4);self.assertEqual(self.n.waypoints.route.target_index,4);self.assertEqual(self.n.waypoints.route.definition(),before);self.assertEqual(self.n.transport.calls,[])
 def test_same_route_id_but_new_pose_rejects_stale_button_payload(self):
  self.prepare();before=self.n.waypoints.route.definition();self.n.waypoints.route.points[4]['pose'][1]=.2
  self.n.waypoints.start_t2(String(data=json.dumps(dict(action='start',request_id='run-start',route_id=before['route_id'],expected_definition=before))))
  self.assertIsNone(self.n.target);self.assertFalse(self.n.waypoints.route.running);self.assertEqual(self.n.transport.calls,[])
 def send(self,**values):self.n.waypoints.start_t2(String(data=json.dumps(values)))
 def test_duplicate_start_never_restarts_and_cancel_only_ends_original_execution(self):
  self.prepare();wp=self.n.waypoints;before=wp.route.definition()
  start=dict(action='start',request_id='run-start',route_id=before['route_id'],expected_definition=before)
  self.send(**start);mission=wp.route.mission_id;self.send(**start)
  self.assertEqual(wp.route.mission_id,mission)
  wp.route.cancel();wp.driver.target=None
  self.send(**start);self.assertFalse(wp.route.running)
  wp.route.start_t2(['point_2'],pose=[5.,0.,0.]);other=wp.route.mission_id
  wp.driver.target=object()
  self.send(action='cancel',request_id='run-stop',start_request_id='run-start',route_id=before['route_id'])
  self.assertEqual(wp.route.mission_id,other);self.assertTrue(wp.route.running);self.assertIsNotNone(wp.driver.target)
  wp.driver.target=None;wp.route.cancel()
 def test_cancel_before_start_tombstones_late_start_and_preserves_draft(self):
  self.prepare();wp=self.n.waypoints;before=wp.route.definition()
  self.send(action='cancel',request_id='early-stop',start_request_id='late-start',route_id=before['route_id'])
  self.send(action='start',request_id='late-start',route_id=before['route_id'],expected_definition=before)
  self.assertFalse(wp.route.running);self.assertIsNone(wp.driver.target);self.assertEqual(wp.route.definition(),before)
 def test_dispatch_failure_does_not_leave_running_ghost_route(self):
  self.prepare();wp=self.n.waypoints;before=wp.route.definition()
  def broken(msg):raise ValueError('dispatch failed')
  wp.driver.accept_special_goal=broken
  self.send(action='start',request_id='bad-dispatch',route_id=before['route_id'],expected_definition=before)
  self.assertFalse(wp.route.running);self.assertEqual(wp.t2_receipts['bad-dispatch'][1]['status'],'rejected')
 def test_cancel_after_route_end_never_stops_new_manual_goal(self):
  self.prepare();w=self.n.waypoints;before=w.route.definition()
  self.send(action='start',request_id='old-start',route_id=before['route_id'],expected_definition=before)
  w.route.cancel();manual=object();self.n.target=manual
  self.send(action='cancel',request_id='old-cancel',start_request_id='old-start',route_id=before['route_id'])
  self.assertIs(self.n.target,manual);self.n.target=None
 def test_wrong_route_cancel_cannot_tombstone_valid_start(self):
  self.prepare();w=self.n.waypoints;before=w.route.definition()
  self.send(action='cancel',request_id='wrong-stop',start_request_id='valid-start',route_id='wrong-route')
  self.send(action='start',request_id='valid-start',route_id=before['route_id'],expected_definition=before)
  self.assertTrue(w.route.running);self.assertIsNotNone(self.n.target)
if __name__=='__main__':unittest.main()
