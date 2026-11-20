"""Private ROS197, actual route/runtime, inert transport and virtual body only."""
import json,math,time,unittest
import numpy as np,rclpy
from std_msgs.msg import String
import test_special_ros as f

class TaskDirectROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 tearDown=f.SpecialROS.tearDown;add=f.SpecialROS.add;worker=f.SpecialROS.worker
 def setUp(self):
  self.token=None;self.gap=False;self.worker_gap=False;self.parked=False;self.statuses=[];self.parked_target=None;self.started=time.monotonic()
  f.SpecialROS.setUp(self);self.hub,self.token=self.worker()
  self.worker_registration={k:self.hub.worker[k] for k in ('worker_id','instance_id','point_ids')}
  original=self.n.status
  def status(value):
   self.statuses.append(value)
   original(value)
   if value.startswith(('paused_navigation','paused_terminal_correction')):
    self.parked=True;self.parked_target=self.n.target.copy()
  self.n.status=status
 def refresh(self):
  f.SpecialROS.refresh(self)
  elapsed=time.monotonic()-self.started
  if self.token and not (self.worker_gap and .2<elapsed<2.7):
   if not self.hub.available_points():self.hub.register(self.worker_registration)
   self.hub.heartbeat(self.token)
  if self.gap and .2<elapsed<.75:self.n.sensor={}
  if (self.parked or elapsed>15) and self.n.target is not None:self.n.cancel('fixture_cancel')
 def start(self,which='point_1',x=.18,yaw=.25,via=False):
  if via:self.add('via',.08)
  self.n.waypoints.add_json(String(data=json.dumps(dict(type='task',point_id=which,pose=[x,0,yaw]))))
  self.n.waypoints.start(None);self.n.seen=self.n.seq
 def test_task_ignores_costmaps_but_reaches_position_and_heading_before_notify(self):
  self.blocked=True;self.refresh();self.start();self.simulate=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertLessEqual(np.linalg.norm(self.position[:2]-[.18,0]),.045)
  self.assertLessEqual(abs(self.position[2]-.25),.09);self.assertFalse(self.parked)
  self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.));self.assertEqual(self.n.mppi_requests,[])
  self.n.cancel('arrived_stopped');self.assertEqual(self.hub.active['point_id'],'point_1')
  self.assertEqual(self.hub.active['status'],'notified');self.assertEqual(self.n.waypoints.route.state,'waiting_task')
 def test_t2_rotates_in_place_and_does_not_use_special_entry_heading(self):
  self.missing=True;self.refresh();self.start('point_2',0.,-.25);self.simulate=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertLessEqual(abs(self.position[2]+.25),.09);self.assertLess(np.linalg.norm(self.position[:2]),.005)
  self.n.cancel('arrived_stopped');self.assertEqual(self.hub.active['point_id'],'point_2')
 def test_normal_last_via_cannot_be_bypassed_through_occupied_maps(self):
  self.blocked=True;self.refresh();self.start(via=True)
  self.assertEqual(self.n.waypoints.route.mode,'navigation');self.assertFalse(self.n._special_execution)
  with self.assertRaisesRegex(ValueError,'blocked'):self.n.checked()
  self.assertIsNone(self.hub.active);self.assertFalse(any(any(c) for c in self.n.transport.calls))
 def test_task_sensor_gap_retains_goal_and_completes_without_grasp_replay(self):
  self.gap=True;self.start();target=self.n.target.copy();identity=self.n.transport.current;self.simulate=True
  self.assertEqual(self.n.run_goal(),'arrived_stopped')
  np.testing.assert_allclose(self.n.target,target);self.assertEqual(self.n.transport.current,identity)
  self.assertIsNone(self.hub.active);self.assertFalse(self.parked)
 def residual(self,which):
  self.position[:]=[.1467,0,.1854];self.refresh();self.start(which,.2,.25)
  self.simulate=False;self.started=time.monotonic()
 def test_stationary_t1_residual_reaches_three_real_stop_reviews_then_fallback(self):
  self.residual('point_1');self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertEqual(self.statuses.count('task_verifying_stopped'),3)
  self.assertIn('arrived_stopped_t1_fallback task_direct',self.statuses)
  self.n.cancel('arrived_stopped');self.assertEqual(self.hub.active['point_id'],'point_1')
 def test_stationary_t2_residual_parks_after_three_checks_without_notification(self):
  self.residual('point_2')
  with self.assertRaisesRegex(ValueError,'goal_changed'):self.n.run_goal()
  self.assertEqual(self.statuses.count('task_verifying_stopped'),3);self.assertIsNone(self.hub.active)
  np.testing.assert_allclose(self.parked_target,[.2,0,.25])
 def test_same_worker_lease_gap_keeps_the_task_entry_and_resumes(self):
  self.worker_gap=True;self.start(x=.5);self.simulate=True
  entry=list(self.n.waypoints.route.direct_source['pose']);identity=self.n.transport.current
  self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertEqual(self.n.waypoints.route.direct_source['pose'],entry)
  self.assertEqual(self.n.transport.current,identity);self.assertFalse(self.parked)
 def test_explicit_cancel_during_task_direct_never_notifies_arrival(self):
  self.start(x=.6);self.simulate=True;self.cancel_after=3
  with self.assertRaisesRegex(ValueError,'goal_changed'):self.n.run_goal()
  self.assertIsNone(self.hub.active);self.assertFalse(self.n.waypoints.route.running)
  self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.))
