import tempfile,unittest,uuid
from pathlib import Path
from task_handoff import HandoffHub,ProtocolError

def ident():return str(uuid.uuid4())
class Handoffs(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.clock=10.;self.path=Path(self.tmp.name)/'journal.json';self.h=HandoffHub(self.path,clock=lambda:self.clock)
  self.worker=dict(worker_id='battery',instance_id=ident(),point_ids=['point_1','point_2'])
  self.token=self.h.register(self.worker)['token']
 def tearDown(self):self.tmp.cleanup()
 def begin(self):return self.h.begin(dict(mission_id=ident(),point_id='point_1',route_point_id='a',map_id='room',goal_pose=[0,0,0],arrival_pose=[0,0,0],pose_stamp_ns=1))
 def event(self,record,event,**extra):return dict(request_id=ident(),mission_id=record['mission_id'],visit_id=record['visit_id'],point_id=record['point_id'],event=event,**extra)
 def test_identity_replay_and_success_stays_pending_until_ack(self):
  a=self.begin();msg=self.event(a,'accept');self.h.event(self.token,msg)
  done=self.event(a,'complete',navigation_ready=True,control_released=True)
  first=self.h.event(self.token,done);self.assertEqual(self.h.event(self.token,done),first)
  self.assertEqual(self.h.active['status'],'resume_pending');self.assertTrue(self.h.blocking())
  with self.assertRaises(ProtocolError):self.h.event(self.token,dict(done,navigation_ready=False))
  self.h.acknowledge(a['visit_id']);self.assertFalse(self.h.blocking())
 def test_wrong_old_visit_does_not_advance(self):
  a=self.begin();b=self.event(a,'accept');b['visit_id']=ident()
  with self.assertRaises(ProtocolError):self.h.event(self.token,b)
  self.assertEqual(self.h.active['status'],'notified')
 def test_cancel_late_complete_never_becomes_resume_pending(self):
  a=self.begin();self.h.event(self.token,self.event(a,'accept'));self.h.cancel()
  with self.assertRaises(ProtocolError):self.h.event(self.token,self.event(a,'complete',navigation_ready=True,control_released=True))
  self.assertEqual(self.h.active['status'],'cancelling')
  self.h.event(self.token,self.event(a,'cancelled',control_released=True));self.assertEqual(self.h.active['status'],'cancelled')
 def test_requires_worker_for_point_and_strict_completion(self):
  a=self.begin()
  for data in (self.event(a,'complete',navigation_ready=True,control_released=True),self.event(a,'accept',unexpected=True)):
   with self.assertRaises(ProtocolError):self.h.event(self.token,data)
  self.h.event(self.token,self.event(a,'accept'))
  with self.assertRaises(ProtocolError):self.h.event(self.token,self.event(a,'complete',navigation_ready=False,control_released=True))
 def test_lease_loss_and_restart_never_automatically_replay(self):
  a=self.begin();self.h.event(self.token,self.event(a,'accept'));self.clock+=2.1;self.h.tick()
  self.assertEqual(self.h.active['status'],'needs_attention');self.assertTrue(self.h.blocking())
  new=HandoffHub(self.path,clock=lambda:self.clock);self.assertEqual(new.active['status'],'needs_attention')
  with self.assertRaises(ProtocolError):new.register(dict(self.worker,instance_id=ident()))
 def test_expired_worker_blocks_new_task_and_no_token_in_public_or_journal(self):
  self.assertNotIn(self.token,str(self.h.state()));self.assertNotIn(self.token,self.path.read_text())
  self.clock+=2.1;self.assertEqual(self.h.available_points(),set())
  with self.assertRaises(ProtocolError):self.begin()
 def test_heartbeat_renews_connection_without_marking_success(self):
  a=self.begin();self.clock+=1.;self.h.heartbeat(self.token);self.clock+=1.5
  self.assertEqual(self.h.poll(self.token)['status'],'notified')
 def test_reconnect_before_timer_cannot_hide_expired_lease(self):
  a=self.begin();self.h.event(self.token,self.event(a,'accept'));self.clock+=2.1
  self.h.register(self.worker)
  with self.assertRaises(ProtocolError):self.h.event(self.token,self.event(a,'complete',navigation_ready=True,control_released=True))
  self.assertEqual(self.h.active['status'],'needs_attention')
 def test_same_worker_can_reconnect_to_cancel_but_never_resume_uncertain_work(self):
  a=self.begin();self.h.event(self.token,self.event(a,'accept'));self.clock+=2.1;self.h.tick()
  renewed=self.h.register(self.worker);self.assertEqual(renewed['token'],self.token)
  self.assertEqual(self.h.active['status'],'needs_attention')
  with self.assertRaises(ProtocolError):self.h.event(self.token,self.event(a,'complete',navigation_ready=True,control_released=True))
  self.h.cancel();self.h.event(self.token,self.event(a,'cancelled',control_released=True));self.h.acknowledge(a['visit_id'])
  self.assertFalse(self.h.blocking())
 def test_new_mission_has_new_visit_and_task_timeout(self):
  a=self.begin();self.h.event(self.token,self.event(a,'accept'));self.h.event(self.token,self.event(a,'complete',navigation_ready=True,control_released=True));self.h.acknowledge(a['visit_id'])
  b=self.begin();self.assertNotEqual(a['visit_id'],b['visit_id']);self.clock+=601;self.h.tick();self.assertEqual(self.h.active['status'],'needs_attention')
if __name__=='__main__':unittest.main()
