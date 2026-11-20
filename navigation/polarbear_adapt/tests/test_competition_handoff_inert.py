"""Loopback HTTP + real navigation state machines; virtual arrivals, no robot.

The worker is a protocol fixture, not a grasp/place implementation. Real-robot
arrival verification and DDS transport are deliberately outside this test.
"""
import tempfile,unittest,uuid
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from mission_route import MissionRoute
from task_handoff import HandoffHub,ProtocolError
from task_client import TaskClient
from task_http import TaskServer
from inert_task_fixture import Driver,Waypoints,attach_runtime,runtime_module

module=runtime_module()
class CompetitionHandoff(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.d=Driver();self.route=MissionRoute('virtual-room-only')
  for p in (dict(type='via',pose=[0,0,0]),dict(type='task',point_id='point_1',pose=[.2,0,0]),dict(type='via',pose=[.6,.2,.1]),dict(type='task',point_id='point_2',pose=[1.2,.2,.1])):self.route.add(p)
  self.h=HandoffHub(Path(self.tmp.name)/'state.json',clock=lambda:self.d.t);self.server=TaskServer(self.h,port=0);self.client=TaskClient('http://127.0.0.1:'+str(self.server.port))
  self.w=Waypoints(self.d,self.route);self.runtime=attach_runtime(module,self.w,self.h);self.w.show()
 def tearDown(self):self.server.close();self.tmp.cleanup()
 def tick(self):
  self.client.heartbeat()
  with patch.object(module.time,'monotonic',lambda:self.d.t):self.runtime.tick()
 def prove_release(self):
  for _ in range(20):self.d.advance();self.tick()
 def arrive_first(self):
  self.client.register();self.route.start(self.h.available_points());self.route.observe([0,0,0],[0,0,0],self.d.source)
  self.route.finish('arrived_stopped',pose=[0,0,0]) # Normal final-via leg precedes task_direct.
  self.d.position[:]=[.2,0,0];self.d.odom[:]=[.2,0,0];self.d.advance();self.route.finish('arrived_stopped');self.runtime.notify(self.route.points[self.route.index]);return self.client.task()
 def test_route_via_T1_external_move_release_then_T2_and_no_automatic_replay(self):
  before=self.client.state();self.assertTrue(before['route']['competition_startable_draft']);self.assertIsNone(before['task']);self.assertFalse(before['worker_ready'])
  first=self.arrive_first();saved=self.route.definition();mission=first['mission_id']
  self.assertEqual(first['point_id'],'point_1');self.assertEqual(first['route_point_id'],saved['points'][1]['id']);self.assertEqual(first['map_id'],'virtual-room-only');self.assertEqual(first['arrival_pose'],[.2,0,0])
  self.client.event(first,'accept',request_id=str(uuid.uuid4()))
  self.d.state['motion_reserved']=True;self.d.position[:]=[.5,.2,.1];self.d.odom[:]=[.5,.2,.1];self.d.advance()
  request=str(uuid.uuid4());reply=self.client.event(first,'complete',request_id=request,navigation_ready=True,control_released=True)
  self.assertEqual(reply['status'],'resume_pending');self.assertEqual(self.w.dispatched,[])
  self.assertEqual(self.client.event(first,'complete',request_id=request,navigation_ready=True,control_released=True),reply)
  self.tick();self.assertEqual(self.h.active['status'],'resume_pending')
  self.d.state['motion_reserved']=False;self.prove_release()
  self.assertEqual(self.h.active['status'],'acknowledged');self.assertEqual(self.w.last_resume_pose,[.5,.2,.1]);self.assertEqual(len(self.w.dispatched),1);self.assertEqual(self.w.dispatched[0]['type'],'via')
  # Duplicate old completion returns its original receipt, never consumes the next via.
  self.client.event(first,'complete',request_id=request,navigation_ready=True,control_released=True);self.assertEqual(self.route.index,2)
  self.route.observe([.5,.2,.1],[.5,.2,.1],self.d.source);self.d.advance(.2);self.route.observe([.55,.2,.1],[.55,.2,.1],self.d.source)
  self.w.dispatch(self.route.finish('arrived_stopped',pose=[.6,.2,.1]))
  self.d.position[:]=[1.2,.2,.1];self.d.odom[:]=[1.2,.2,.1];self.d.advance();self.route.finish('arrived_stopped');self.runtime.notify(self.route.points[self.route.index])
  second=self.client.task();self.assertEqual(second['point_id'],'point_2');self.assertEqual(second['mission_id'],mission);self.assertNotEqual(second['visit_id'],first['visit_id']);self.assertEqual(second['route_point_id'],saved['points'][3]['id'])
  with self.assertRaises(HTTPError):self.client.event(first,'complete',request_id=str(uuid.uuid4()),navigation_ready=True,control_released=True)
  self.client.event(second,'accept',request_id=str(uuid.uuid4()));self.client.event(second,'complete',request_id=str(uuid.uuid4()),navigation_ready=True,control_released=True);self.prove_release()
  self.assertEqual(self.route.state,'completed');self.assertEqual(self.route.index,4);self.assertEqual(self.h.active['status'],'acknowledged');self.assertEqual(len(self.w.dispatched),2)
  self.assertIsNone(self.route.start(self.h.available_points()));self.assertFalse(self.client.state()['route']['competition_startable_draft'])
  self.route.load(saved);self.w.show();self.assertTrue(self.client.state()['route']['competition_startable_draft']);self.assertIsNone(self.route.mission_id)
 def test_read_only_route_checker_does_not_register_worker_or_begin_mission(self):
  import contextlib,io,json
  from check_task_route import main
  output=io.StringIO()
  with contextlib.redirect_stdout(output):code=main(['--url',self.client.url])
  value=json.loads(output.getvalue());self.assertEqual(code,0);self.assertFalse(value['worker_ready']);self.assertEqual(value['route']['task_points'][1]['point_id'],'point_2')
  self.assertIsNone(self.route.mission_id);self.assertIsNone(self.h.active);self.assertFalse(self.h.worker)
 def test_second_worker_cannot_steal_and_cancelled_visit_never_advances(self):
  first=self.arrive_first();other=TaskClient(self.client.url)
  with self.assertRaises(HTTPError):other.register()
  self.client.event(first,'accept',request_id=str(uuid.uuid4()));self.route.cancel();self.h.cancel()
  with self.assertRaises(HTTPError):self.client.event(first,'complete',request_id=str(uuid.uuid4()),navigation_ready=True,control_released=True)
  self.client.event(first,'cancelled',request_id=str(uuid.uuid4()),control_released=True);self.prove_release()
  self.assertEqual(self.h.active['outcome'],'cancelled');self.assertFalse(self.route.running);self.assertEqual(self.w.dispatched,[])
 def test_journal_restart_keeps_uncertain_work_paused_not_replayed(self):
  first=self.arrive_first();self.client.event(first,'accept',request_id=str(uuid.uuid4()))
  restored=HandoffHub(self.h.path,clock=lambda:self.d.t)
  self.assertEqual(restored.state()['task']['status'],'needs_attention');self.assertEqual(restored.state()['task']['visit_id'],first['visit_id']);self.assertFalse(restored.available_points())
  with self.assertRaises(ProtocolError):restored.register(dict(worker_id='battery',instance_id=str(uuid.uuid4()),point_ids=['point_1','point_2']))
  self.assertEqual(self.w.dispatched,[])
if __name__=='__main__':unittest.main()
