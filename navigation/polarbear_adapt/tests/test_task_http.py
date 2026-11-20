import json,tempfile,unittest,uuid
from pathlib import Path
from urllib.request import Request,urlopen
from urllib.error import HTTPError
from task_handoff import HandoffHub
from task_http import TaskServer
from task_client import TaskClient

class Http(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.h=HandoffHub(Path(self.tmp.name)/'state.json');self.s=TaskServer(self.h,port=0);self.url='http://127.0.0.1:'+str(self.s.port)
 def tearDown(self):self.s.close();self.tmp.cleanup()
 def test_client_registration_and_exact_task_handshake(self):
  c=TaskClient(self.url);c.register(['point_1']);c.heartbeat();self.assertEqual(self.h.available_points(),{'point_1'})
  a=self.h.begin(dict(mission_id=str(uuid.uuid4()),point_id='point_1',route_point_id='a',map_id='r',goal_pose=[0,0,0],arrival_pose=[0,0,0],pose_stamp_ns=1))
  task=c.task();self.assertEqual(a['visit_id'],task['visit_id'])
  c.event(task,'accept',request_id=str(uuid.uuid4()));c.event(task,'complete',request_id=str(uuid.uuid4()),navigation_ready=True,control_released=True)
  self.assertEqual(self.h.active['status'],'resume_pending')
 def test_auth_unknown_route_bad_json_and_size(self):
  with self.assertRaises(HTTPError) as got:urlopen(self.url+'/v1/task')
  self.assertEqual(got.exception.code,403)
  for path,body,code in [('/v1/worker',b'{"a":NaN}',400),('/v1/worker',b'x'*32769,413),('/other',b'{}',404)]:
   with self.assertRaises(HTTPError) as got:urlopen(Request(self.url+path,body,{'Content-Type':'application/json'}))
   self.assertEqual(got.exception.code,code)
 def test_state_never_exposes_token_and_bind_is_local(self):
  c=TaskClient(self.url);c.register(['point_2'])
  with urlopen(self.url+'/v1/state') as r:v=r.read().decode()
  self.assertNotIn(c.token,v);self.assertEqual(self.s.http.server_address[0],'127.0.0.1')
 def test_uncertain_worker_failure_does_not_execute_any_callback(self):
  c=TaskClient(self.url);c.register(['point_1']);self.assertIsNone(c.task())
  self.assertIsNone(self.h.active)
if __name__=='__main__':unittest.main()
