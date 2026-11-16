"""Real runtime policy with inert sensor/ROS edges; never sends a command."""
import tempfile,unittest,uuid
from pathlib import Path
from unittest.mock import patch
from mission_route import MissionRoute
from task_handoff import HandoffHub
from inert_task_fixture import Driver,Waypoints,attach_runtime,runtime_module

module=runtime_module()
class RuntimeHandoff(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.d=Driver();self.route=MissionRoute('room');self.route.add(dict(type='task',point_id='point_1',pose=[0.,0.,0.]));self.route.add(dict(type='via',pose=[1.,0.,0.]))
  self.h=HandoffHub(Path(self.tmp.name)/'task.json',clock=lambda:self.d.t);self.token=self.h.register(dict(worker_id='battery',instance_id=str(uuid.uuid4()),point_ids=['point_1','point_2']))['token']
  self.w=Waypoints(self.d,self.route);self.r=attach_runtime(module,self.w,self.h);self.route.start(self.h.available_points());self.route.finish('arrived_stopped');self.r.notify(self.route.points[0])
 def tearDown(self):self.tmp.cleanup()
 def event(self,event,**v):
  a=self.h.active;p={k:a[k] for k in ('mission_id','visit_id','point_id')};p.update(request_id=str(uuid.uuid4()),event=event,**v);return self.h.event(self.token,p)
 def complete(self):self.event('accept');self.event('complete',control_released=True,navigation_ready=True)
 def tick(self):
  self.h.heartbeat(self.token)
  with patch.object(module.time,'monotonic',lambda:self.d.t):self.r.tick()
 def prepare_final(self,mutation):
  self.complete();self.tick()
  # Set up a satisfied proof so only the final-read boundary changes.
  self.r.stop.add=lambda *a:True;self.r.window.observe=lambda *a:True
  self.d.advance()
  mutate_at=self.d.console_reads+2
  def on_read(count):
   if count==mutate_at:mutation()
  self.d.on_console=on_read
  self.tick()
 def test_pose_expires_during_final_console_read_does_not_acknowledge(self):
  self.prepare_final(lambda:setattr(self.d,'t',self.d.t+.35))
  self.assertEqual(self.h.active['status'],'resume_pending');self.assertEqual(self.w.dispatched,[]);self.assertEqual(self.route.index,0)
 def test_pose_revoked_during_final_console_read_does_not_acknowledge(self):
  self.prepare_final(lambda:self.d.lease.withdraw())
  self.assertEqual(self.h.active['status'],'resume_pending');self.assertEqual(self.w.dispatched,[])
 def test_new_pose_during_final_console_read_requires_new_proof(self):
  self.prepare_final(lambda:self.d.advance(.01))
  self.assertEqual(self.h.active['status'],'resume_pending');self.assertEqual(self.w.dispatched,[])
 def test_complete_is_pending_until_actual_release_stop_and_new_pose_verified(self):
  self.d.position[:]=[.4,.2,.3];self.d.odom[:]=[.4,.2,.3];self.d.state['motion_reserved']=True;self.complete()
  for _ in range(15):self.d.advance();self.tick()
  self.assertEqual(self.h.active['status'],'resume_pending');self.assertEqual(self.w.dispatched,[])
  self.d.state['motion_reserved']=False
  for _ in range(20):self.d.advance();self.tick()
  self.assertEqual(self.h.active['status'],'acknowledged');self.assertEqual(self.w.dispatched[0]['pose'],[1.,0.,0.]);self.assertEqual(self.w.last_resume_pose,[.4,.2,.3])
 def test_arm_token_alone_does_not_block_resume_but_active_chassis_does(self):
  self.d.state['motion_reserved']=True;self.d.state['chassis_active']=True;self.complete()
  for _ in range(40):self.d.advance();self.tick()
  self.assertEqual(self.h.active['status'],'resume_pending')
  self.d.state['chassis_active']=False
  for _ in range(40):self.d.advance();self.tick()
  self.assertTrue(self.d.state['motion_reserved'])
  self.assertEqual(self.h.active['status'],'acknowledged')
 def test_closed_execution_gate_does_not_acknowledge_or_consume_task(self):
  self.d.ready_file=Path(self.tmp.name)/'missing-drive-ready'
  self.complete()
  for _ in range(20):self.d.advance();self.tick()
  self.assertEqual(self.h.active['status'],'resume_pending');self.assertEqual(self.route.index,0);self.assertEqual(self.w.dispatched,[])
 def test_execution_gate_closes_during_final_read_does_not_acknowledge(self):
  gate=Path(self.tmp.name)/'drive-ready';gate.touch();self.d.ready_file=gate
  self.prepare_final(gate.unlink)
  self.assertEqual(self.h.active['status'],'resume_pending');self.assertEqual(self.w.dispatched,[])
 def test_occupied_footprint_or_stale_inputs_never_acknowledge(self):
  for field,value in (('blocked',True),('stale',['/pb/scan'])):
   setattr(self.d,field,value);self.complete() if self.h.active['status']=='notified' else None
   for _ in range(15):self.d.advance();self.tick()
   self.assertEqual(self.h.active['status'],'resume_pending');self.assertEqual(self.w.dispatched,[])
   setattr(self.d,field,False if field=='blocked' else [])
 def test_notification_uses_one_authoritative_batch_without_second_pose_lookup(self):
  self.h.cancel();self.h.operator_cancel_verified(self.h.active['visit_id']);self.h.acknowledge(self.h.active['visit_id'])
  self.d.pose=lambda *args:(_ for _ in ()).throw(ValueError('unrelated second lookup'))
  self.r.notify(self.route.points[0]);self.assertEqual(self.h.active['arrival_pose'],[0.,0.,0.])

class FinalDirectTask(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.d=Driver();self.route=MissionRoute('room')
  self.route.add(dict(type='task',point_id='point_2',pose=[0.,0.,0.]))
  self.h=HandoffHub(Path(self.tmp.name)/'task.json',clock=lambda:self.d.t)
  self.token=self.h.register(dict(worker_id='battery',instance_id=str(uuid.uuid4()),point_ids=['point_1','point_2']))['token']
  self.w=Waypoints(self.d,self.route);self.r=attach_runtime(module,self.w,self.h)
  self.route.start(self.h.available_points());self.route.finish('arrived_stopped');self.r.notify(self.route.points[0])
  self.d.blocked=True
 tearDown=RuntimeHandoff.tearDown;event=RuntimeHandoff.event;complete=RuntimeHandoff.complete;tick=RuntimeHandoff.tick
 def test_final_task_can_finish_without_starting_navigation_through_occupied_map(self):
  self.complete()
  for _ in range(20):self.d.advance();self.tick()
  self.assertEqual(self.h.active['status'],'acknowledged');self.assertEqual(self.route.state,'completed')
  self.assertEqual(self.w.dispatched,[])
 def test_final_task_still_requires_idle_chassis_and_valid_pose(self):
  self.complete();self.d.state['chassis_active']=True
  for _ in range(20):self.d.advance();self.tick()
  self.assertEqual(self.h.active['status'],'resume_pending')
  self.d.state['chassis_active']=False
  for _ in range(20):
   self.d.advance();self.d.lease.withdraw();self.tick()
  self.assertEqual(self.h.active['status'],'resume_pending');self.assertEqual(self.w.dispatched,[])
if __name__=='__main__':unittest.main()
