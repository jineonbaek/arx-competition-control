"""Competition diagnostics must not change general route execution semantics."""
import copy,tempfile,unittest,uuid
from pathlib import Path
from mission_route import MissionRoute
from task_handoff import HandoffHub

class TaskRouteReadiness(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.h=HandoffHub(Path(self.tmp.name)/'journal.json');self.r=MissionRoute('room-A')
 def tearDown(self):self.tmp.cleanup()
 def add(self,kind,x,tag=None,name=None):
  v=dict(type=kind,pose=[x,.25,.7])
  if tag:v['point_id']=tag
  if name:v['name']=name
  self.r.add(v)
 def publish(self):
  self.h.set_route(self.r.definition(),state=self.r.state,running=self.r.running,index=self.r.index)
  return self.h.state()['route']
 def test_named_via_and_special_cannot_impersonate_task_points(self):
  self.add('via',1,name='T1');self.add('special',2,name='T2');v=self.publish()
  self.assertFalse(v['competition_config_valid']);self.assertFalse(v['competition_startable_draft']);self.assertEqual(v['task_points'],[])
 def test_tasks_keep_identity_pose_order_and_map_across_draft_load(self):
  self.add('via',.2);self.add('task',1,'point_1');self.add('special',1.5);self.add('task',2,'point_2')
  saved=self.r.definition();self.r.load(saved);v=self.publish()
  self.assertTrue(v['competition_config_valid']);self.assertTrue(v['competition_startable_draft'])
  self.assertEqual(v['route_id'],saved['route_id']);self.assertEqual(v['map_id'],'room-A')
  self.assertEqual([(p['index'],p['point_id'],p['route_point_id'],p['pose']) for p in v['task_points']],[(1,'point_1',saved['points'][1]['id'],[1.,.25,.7]),(3,'point_2',saved['points'][3]['id'],[2.,.25,.7])])
 def test_missing_duplicate_and_reversed_are_not_competition_routes(self):
  for ids in (('point_1',),('point_2','point_1'),('point_1','point_1','point_2')):
   self.r.reset()
   for i,tag in enumerate(ids):self.add('task',i,tag)
   self.assertFalse(self.publish()['competition_config_valid'])
  # Generic mission semantics still allow repeated visits; diagnosis does not change them.
  self.r.start({'point_1','point_2'});self.assertTrue(self.r.running)
 def test_completed_cancelled_and_running_require_explicit_draft_reload(self):
  self.add('task',1,'point_1');self.add('task',2,'point_2');saved=self.r.definition()
  for state,running in (('completed',False),('cancelled',False),('running',True),('waiting_task',True)):
   self.r.state=state;self.r.running=running;v=self.publish()
   self.assertTrue(v['competition_config_valid']);self.assertFalse(v['competition_startable_draft'])
  self.r.running=False;self.r.load(saved);self.assertTrue(self.publish()['competition_startable_draft']);self.assertIsNone(self.r.mission_id)
 def test_public_snapshot_is_detached_and_reading_creates_no_task(self):
  self.add('task',1,'point_1');self.add('task',2,'point_2');definition=self.r.definition()
  self.h.set_route(definition,state='draft',running=False,index=0);definition['points'][0]['pose'][0]=99
  v=self.h.state();v['route']['task_points'][0]['pose'][0]=88
  self.assertEqual(self.h.state()['route']['task_points'][0]['pose'][0],1.)
  self.assertFalse(self.h.state()['worker_ready']);self.assertIsNone(self.h.state()['task']);self.assertIsNone(self.r.mission_id)
 def test_state_before_ros_route_is_available_does_not_claim_ready(self):
  self.assertIsNone(self.h.state()['route'])
if __name__=='__main__':unittest.main()
