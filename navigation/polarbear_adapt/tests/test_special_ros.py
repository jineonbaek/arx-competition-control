"""Private ROS197; real navigation objects with inert physical transport."""
import io,json,math,time,unittest,os
assert os.environ.get('ROS_DOMAIN_ID') in ('197','198')
import numpy as np,rclpy
from geometry_msgs.msg import PoseStamped,TransformStamped,PoseArray,PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from path_audit import AuditGrid
import test_mission_ros as fixtures

class SpecialROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 tearDown=fixtures.MissionROS.tearDown
 add=fixtures.MissionROS.add
 worker=fixtures.MissionROS.worker
 event=fixtures.MissionROS.event
 def setUp(self):
  self.simulate=False;self.blocked=False;self.missing=False;self.cancel_after=None;self.external_after=None;self.external_epoch=0;self.last_physical=time.monotonic()
  fixtures.MissionROS.setUp(self)
  def snapshot(*a,**kw):return io.BytesIO(json.dumps(dict(execution_enabled=True,nav_sdk_ready=True,body=dict(fresh=True),chassis_active=False,motion_reserved=self.n.transport.reserved,capabilities=dict(chassis=''),command_epoch=1+len(self.n.transport.calls)+self.external_epoch)).encode())
  self.n.transport.open=snapshot
 def refresh(self):
  now=time.monotonic();dt=now-self.last_physical;self.last_physical=now
  if self.simulate and self.n.transport.calls:
   x,y,w=self.n.transport.calls[-1];c,s=math.cos(self.position[2]),math.sin(self.position[2]);self.position+=dt*np.array([c*x-s*y,s*x+c*y,w])
  fixtures.MissionROS.refresh(self)
  if self.blocked:
   st=self.n.get_clock().now().to_msg();g=AuditGrid(np.full((200,200),100),[-5,-5],.05,self.n.poly);self.n.grids={f:(st,g) for f in ('map','odom')}
  if self.missing:self.n.grids={}
  if self.cancel_after and len(self.n.transport.calls)>=self.cancel_after:self.cancel_after=None;self.n.cancel('cancelled')
  if self.external_after and len(self.n.transport.calls)>=self.external_after:self.external_after=None;self.external_epoch+=1;self.simulate=False
 def start_special(self,end=.2,mid=None):
  self.add('special',0.)
  if mid is not None:self.add('special',mid)
  self.add('special',end);self.n.waypoints.start(None);self.n.seen=self.n.seq
 def test_current_pose_named_capture_is_idempotent_and_never_starts(self):
  self.position=np.array([.4,.2,.3]);self.refresh();m=String(data=json.dumps(dict(request_id='capture-1',kind='special',name='入口')))
  self.n.waypoints.add_current(m);self.n.waypoints.add_current(m)
  ps=self.n.waypoints.route.points;self.assertEqual(len(ps),1);np.testing.assert_allclose(ps[0]['pose'],self.position)
  self.assertEqual(ps[0]['name'],'入口');self.assertIsNone(self.n.target);self.assertEqual(self.n.transport.calls,[])
 def test_legacy_untyped_add_keeps_stop_semantics_without_starting(self):
  m=PoseStamped();m.header.frame_id='map';m.pose.orientation.w=1.;self.n.waypoints.add(m)
  self.assertEqual(self.n.waypoints.route.points[0]['type'],'stop');self.assertIsNone(self.n.target)
 def test_current_pose_missing_authority_and_active_route_rejected(self):
  self.n.tf.clear();invalid=PoseArray();invalid.header.frame_id='map';self.n.receive_chassis_snapshot(invalid)
  self.n.waypoints.add_current(String(data=json.dumps(dict(request_id='missing',kind='via'))));self.assertEqual(self.n.waypoints.route.points,[])
  self.refresh();self.start_special();before=len(self.n.waypoints.route.points)
  self.n.waypoints.add_current(String(data=json.dumps(dict(request_id='active',kind='special'))));self.assertEqual(len(self.n.waypoints.route.points),before)
 def test_authority_withdrawal_blocks_capture_despite_fresh_raw_tf(self):
  invalid=PoseArray();invalid.header.frame_id='map';invalid.header.stamp=self.n.get_clock().now().to_msg();self.n.receive_chassis_snapshot(invalid)
  self.n.waypoints.add_current(String(data=json.dumps(dict(request_id='invalid-authority',kind='special'))))
  self.assertEqual(self.n.waypoints.route.points,[])
 def test_relocalization_rejects_delayed_valid_snapshot_and_old_tf(self):
  old=self.n.pose_snapshot
  m=PoseWithCovarianceStamped();m.header.frame_id='map';self.n.relocalizing(m);self.n.receive_chassis_snapshot(old)
  self.n.waypoints.add_current(String(data=json.dumps(dict(request_id='relocalizing',kind='via'))));self.assertEqual(self.n.waypoints.route.points,[])
 def test_expired_authority_snapshot_cannot_be_replaced_with_fresh_tf(self):
  from copy import deepcopy
  old=deepcopy(self.n.pose_snapshot);old.header.stamp.sec-=1;self.n.receive_chassis_snapshot(old)
  self.n.waypoints.add_current(String(data=json.dumps(dict(request_id='expired',kind='via'))));self.assertEqual(self.n.waypoints.route.points,[])
 def test_bottom_mark_watchdog_is_retained_separate_from_maps(self):
  self.n.require_bottom_mark=True;self.n.bottom_mark_stamp=self.n.now()-100.
  with self.assertRaisesRegex(ValueError,'bottom_obstacles_stale'):self.n.special_poses()
  self.n.bottom_mark_stamp=self.n.now();self.n.special_poses()
 def test_occupied_costmaps_do_not_stop_special_and_terminal_stops(self):
  self.blocked=True;self.refresh();self.start_special();self.simulate=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped');self.assertTrue(any(c[0]>0 for c in self.n.transport.calls));self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.))
  self.assertLessEqual(abs(self.position[0]-.2),.10);self.n.cancel('arrived_stopped');self.assertEqual(self.n.waypoints.route.state,'completed')
 def test_absent_maps_do_not_stop_special_and_midpoint_has_no_brake(self):
  self.missing=True;self.refresh();self.start_special(.3,mid=.15);self.simulate=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped');calls=self.n.transport.calls;first=next(i for i,c in enumerate(calls) if not any(c))
  self.assertTrue(all(not any(c) for c in calls[first:]));self.assertEqual(self.n.waypoints.route.index,2)
 def test_foreign_arrows_empty_paths_and_publish_point_cannot_seize_special(self):
  self.start_special();target=self.n.target.copy();seq=self.n.seq
  m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.n.get_clock().now().to_msg();m.pose.position.x=8.;m.pose.orientation.w=1.
  self.n.plan_changed(m);self.n.receive(Path());self.n.changed(m)
  np.testing.assert_allclose(self.n.target,target);self.assertEqual(self.n.seq,seq);self.assertEqual(self.n.transport.calls,[])
 def test_explicit_cancel_stops_without_advancing(self):
  self.start_special(.5);self.simulate=True;self.cancel_after=3
  with self.assertRaises(ValueError):self.n.run_goal()
  self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.));self.assertFalse(self.n.waypoints.route.running);self.assertNotEqual(self.n.waypoints.route.state,'completed')
 def test_stale_sensor_still_blocks_special(self):
  self.start_special();self.n.sensor={}
  with self.assertRaisesRegex(ValueError,'stale'):self.n.special_poses()
  self.assertFalse(any(any(c) for c in self.n.transport.calls))
 def test_source_moved_before_execution_cannot_start_blind_segment(self):
  self.start_special(.5);self.position[0]=.3;self.refresh();self.simulate=True
  from special_runtime import run_special
  with self.assertRaisesRegex(ValueError,'entry'):run_special(self.n)
  self.assertEqual(self.n.transport.calls,[])
 def test_special_never_falls_back_to_legacy_scaled_sdk_channel(self):
  self.start_special();self.simulate=True;self.n.transport.native_sdk=False
  from special_runtime import run_special
  with self.assertRaisesRegex(ValueError,'native_sdk'):run_special(self.n)
  self.assertEqual(self.n.transport.calls,[])
 def test_unrelated_epoch_does_not_cancel_but_real_no_progress_still_stops(self):
  self.start_special(.5);self.simulate=True;self.external_after=2
  from special_runtime import run_special
  with self.assertRaisesRegex(ValueError,'special_no_progress'):run_special(self.n)
  self.assertGreater(len(self.n.transport.calls),2);self.assertFalse(self.n.active)
  self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.))
 def test_uncancelled_old_mppi_goal_blocks_special_output(self):
  from rclpy.task import Future
  class Old:
   status=2;accepted=True
   def cancel_goal_async(self):return Future()
   def get_result_async(self):return Future()
  self.start_special();self.n.mppi_handle=Old()
  from special_runtime import cancel_mppi
  with self.assertRaisesRegex(ValueError,'mppi'):cancel_mppi(self.n)
  self.assertEqual(self.n.transport.calls,[])
 def test_failed_old_request_does_not_poison_normal_request_tracking(self):
  from rclpy.task import Future
  failed=Future();failed.set_exception(RuntimeError('old request uncertain'));pending=Future();self.n.mppi_requests=[failed]
  self.n.remember_mppi_request(pending);self.assertIn(pending,self.n.mppi_requests);self.assertIn(failed,self.n.mppi_requests)
 def test_task_release_near_special_entry_ignores_maps_but_requires_complete_and_stop(self):
  h,t=self.worker();self.add('task',0.,'point_1');self.add('special',0.);self.add('special',.2);self.n.waypoints.start(None);self.n.seen=self.n.seq;self.n.cancel('arrived_stopped')
  self.missing=True;self.refresh();self.n.spin(.1);self.assertIsNone(self.n.target)
  self.event(h,t,'accept');self.event(h,t,'complete',navigation_ready=True,control_released=True)
  until=time.monotonic()+3.
  while self.n.target is None and time.monotonic()<until:h.heartbeat(t);self.n.spin(.08)
  self.assertIsNotNone(self.n.target);self.assertEqual(h.active['status'],'acknowledged');self.assertEqual(self.n.waypoints.route.mode,'direct');self.assertEqual(self.n.transport.calls,[])
 def test_far_special_source_retains_normal_occupied_handoff_gate(self):
  h,t=self.worker();self.add('task',0.,'point_1');self.add('special',1.);self.add('special',2.);self.n.waypoints.start(None);self.n.seen=self.n.seq;self.n.cancel('arrived_stopped')
  self.blocked=True;self.refresh();self.event(h,t,'accept');self.event(h,t,'complete',navigation_ready=True,control_released=True);self.n.spin(.3)
  self.assertIsNone(self.n.target);self.assertEqual(h.active['status'],'resume_pending');self.assertEqual(self.n.transport.calls,[])
 def test_normal_navigation_still_rejects_occupied_footprint(self):
  self.add('stop',.2);self.n.waypoints.start(None);self.n.seen=self.n.seq;self.blocked=True;self.refresh()
  with self.assertRaisesRegex(ValueError,'blocked'):self.n.checked()

if __name__=='__main__':unittest.main()
