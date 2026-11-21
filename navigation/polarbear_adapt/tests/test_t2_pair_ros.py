import os,json,unittest,numpy as np,rclpy
assert os.environ.get('ROS_DOMAIN_ID')=='197'
from std_msgs.msg import String
import test_special_ros as fixture
from test_t2_scope_ros import ScopeROS
class PairROS(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init());tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=fixture.SpecialROS.setUp;tearDown=fixture.SpecialROS.tearDown;refresh=fixture.SpecialROS.refresh;add=fixture.SpecialROS.add
 prepare=ScopeROS.prepare
 def test_far_two_point_start_uses_special_motion_with_occupied_maps(self):
  self.prepare();h=self.n.waypoints.hub;token=h.worker['token'];self.n.create_timer(.2,lambda:h.heartbeat(token))
  r=self.n.waypoints.route;r.points=r.points[-2:]
  r.points[0]['pose']=[.25,0.,0.];r.points[1]['pose']=[.5,0.,0.]
  self.position=np.array([0.,0.,0.]);self.blocked=True;self.refresh();before=r.definition()
  self.n.waypoints.start_t2(String(data=json.dumps(dict(action='start',request_id='pair-start',route_id=before['route_id'],expected_definition=before))))
  self.assertIsNotNone(self.n.target);self.assertEqual(r.mode,'direct');self.assertEqual(r.target_index,0)
  np.testing.assert_allclose(r.direct_source['pose'],[0.,0.,0.]);self.assertEqual(self.n.transport.calls,[])
  self.n.seen=self.n.seq;self.simulate=True;self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertTrue(any(c[0]>0 for c in self.n.transport.calls));self.assertEqual(self.n.transport.calls[-1],(0.,0.,0.))
  self.n.cancel('arrived_stopped');self.assertEqual(r.mode,'task_direct');self.assertEqual(r.target_index,1)
  self.assertEqual(r.definition(),before)
 def test_worker_checks_only_tasks_inside_execution_range(self):
  self.prepare();r=self.n.waypoints.route;later=dict(r.points[1],id='later-t1')
  r.points=r.points[-2:]+[later];r.execution_end=2;r.t2_index=1;r.running=True;r.mission_id='pair-worker'
  self.n.bind_task_worker();self.assertTrue(self.n.task_worker_ready())
 def check_middle_t2_receipt(self,suffix,stale_maps):
  original_stale=self.n.stale_inputs
  self.prepare();w=self.n.waypoints;r=w.route;h=w.hub
  r.points=r.points[-2:]+[dict(id='after',type=suffix,pose=[.5,0.,0.])]
  r.points[0]['pose']=[0.,0.,0.];r.points[1]['pose']=[.25,0.,0.];before=r.definition()
  r.start_t2(['point_2'],[0.,0.,0.]);r.finish('arrived_stopped',[0.,0.,0.]);r.finish('arrived_stopped',[.25,0.,0.]);w.show()
  self.position=np.array([.25,0.,0.]);self.blocked=True;self.refresh()
  h.begin(dict(mission_id=r.mission_id,point_id='point_2',route_point_id=r.points[1]['id'],map_id=r.map_id,goal_pose=[.25,0.,0.],arrival_pose=[.25,0.,0.],pose_stamp_ns=self.n.pose_snapshot.header.stamp.sec*10**9+self.n.pose_snapshot.header.stamp.nanosec))
  token=h.worker['token']
  import uuid
  def event(kind,**values):return h.event(token,dict(mission_id=h.active['mission_id'],visit_id=h.active['visit_id'],point_id='point_2',event=kind,request_id=str(uuid.uuid4()),**values))
  event('accept');event('complete',navigation_ready=True,control_released=True)
  self.n.stale_inputs=(lambda:['global_costmap','local_costmap']) if stale_maps else original_stale
  rt=w.runtime;rt.tick();rt.stop.add=lambda *a:True;rt.window.observe=lambda *a:True;rt.stamp=None;rt.tick()
  if suffix!='special':
   self.assertEqual(h.active['status'],'resume_pending');self.assertEqual(r.state,'waiting_task');self.assertIsNone(self.n.target)
   self.blocked=False;self.n.stale_inputs=original_stale;self.refresh();rt.stop.add=lambda *a:True;rt.window.observe=lambda *a:True;rt.stamp=None;rt.tick()
  self.assertEqual(h.active['status'],'acknowledged',str(h.state()));self.assertEqual(r.state,'running');self.assertTrue(r.running)
  self.assertEqual(r.index,2);self.assertIsNotNone(self.n.target);self.assertEqual(r.definition(),before);self.assertEqual(self.n.transport.calls,[])
  self.assertEqual(r.required_task_ids(),set())
  self.assertEqual(r.mode,'direct' if suffix=='special' else 'navigation')
  return before
 def test_special_suffix_resumes_despite_occupied_maps_and_finishes_only_at_suffix(self):
  before=self.check_middle_t2_receipt('special',False);r=self.n.waypoints.route
  self.n.seen=self.n.seq;self.simulate=True;self.assertEqual(self.n.run_goal(),'arrived_stopped');self.n.cancel('arrived_stopped')
  self.assertEqual(r.state,'completed');self.assertEqual(r.definition(),before)
 def test_special_suffix_resumes_with_stale_maps(self):self.check_middle_t2_receipt('special',True)
 def test_stop_suffix_waits_for_free_maps_then_dispatches(self):self.check_middle_t2_receipt('stop',False)
 def test_stop_suffix_waits_for_fresh_maps_then_dispatches(self):self.check_middle_t2_receipt('stop',True)
 def test_via_suffix_waits_for_free_maps_then_dispatches(self):self.check_middle_t2_receipt('via',False)
 def test_via_suffix_waits_for_fresh_maps_then_dispatches(self):self.check_middle_t2_receipt('via',True)
if __name__=='__main__':unittest.main()
