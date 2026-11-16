"""Actual handoff/session protocol, no controller or actuator connection."""
import sys,tempfile,threading,unittest,uuid
from pathlib import Path
from types import SimpleNamespace
from navigation_session import NavigationSession
from task_handoff import HandoffHub

class ReleaseWait(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.h=HandoffHub(Path(self.tmp.name)/'visit.json')
  token=self.h.register(dict(worker_id='battery',instance_id=str(uuid.uuid4()),point_ids=['point_1','point_2']))['token']
  self.visit=self.h.begin(dict(mission_id=str(uuid.uuid4()),point_id='point_1',route_point_id='t1',map_id='room',goal_pose=[0,0,0],arrival_pose=[0,0,0],pose_stamp_ns=1))
  self.h.event(token,dict(request_id=str(uuid.uuid4()),mission_id=self.visit['mission_id'],visit_id=self.visit['visit_id'],point_id='point_1',event='accept'))
  self.busy=True;self.reads=0
  def snapshot():self.reads+=1;return dict(chassis_active=self.busy)
  def event(visit,event,request_id,**kw):return self.h.event(token,dict(request_id=request_id,mission_id=visit['mission_id'],visit_id=visit['visit_id'],point_id=visit['point_id'],event=event,**kw))
  self.s=NavigationSession(SimpleNamespace(snapshot=snapshot),None,client=SimpleNamespace(event=event))
  self.context=dict(run_id='run',cancelled=threading.Event());self.s.context=self.context
  self.s.active=True;self.s.visit=self.visit;self.s.latest=self.h.state()
 def tearDown(self):self.tmp.cleanup()
 def test_released_chassis_completes_original_visit_without_replaying_work(self):
  timer=threading.Timer(.15,lambda:setattr(self,'busy',False));timer.start()
  try:self.s.complete_visit(self.context)
  finally:timer.join()
  self.assertGreaterEqual(self.reads,2)
  self.assertEqual(self.h.active['visit_id'],self.visit['visit_id'])
  self.assertEqual(self.h.active['status'],'resume_pending')
  self.assertEqual(sum(e['event']=='complete' for e in self.h.events),1)
 def test_explicit_stop_while_waiting_never_reports_completed(self):
  timer=threading.Timer(.12,self.context['cancelled'].set);timer.start()
  try:
   with self.assertRaisesRegex(ValueError,'整场停止'):self.s.complete_visit(self.context)
  finally:timer.join()
  self.assertEqual(self.h.active['status'],'running')
  self.assertFalse(any(e['event']=='complete' for e in self.h.events))
 def test_uncertain_work_result_is_never_forged_when_chassis_releases(self):
  self.busy=False;self.s.latest['task']['status']='needs_attention'
  timer=threading.Timer(.12,self.context['cancelled'].set);timer.start()
  try:
   with self.assertRaisesRegex(ValueError,'整场停止'):self.s.complete_visit(self.context)
  finally:timer.join()
  self.assertEqual(self.h.active['status'],'running')
