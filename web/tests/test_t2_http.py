import json,threading,unittest
from unittest.mock import Mock
from http.server import ThreadingHTTPServer
import test_battery_task_http as helper
from server import make_handler
from test_control import Controller,FakeAdapter,configured
class T2HTTP(unittest.TestCase):
 request=staticmethod(helper.BatteryTaskHTTPTests.request);session=helper.BatteryTaskHTTPTests.session
 def test_independent_page_session_endpoint_and_origin_with_no_robot_writes(self):
  adapter=FakeAdapter();c=configured();c['chassis_backend']='local';ctrl=Controller(c,adapter);task=Mock();task.snapshot.return_value=dict(status='idle',tasks=[],scope='t2_test');task.action.return_value=dict(status='running',scope='t2_test')
  server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(ctrl,t2_test_task=task));server.daemon_threads=True;server.stopping=threading.Event();thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
  self.addCleanup(thread.join);self.addCleanup(server.server_close);self.addCleanup(server.shutdown);self.addCleanup(server.stopping.set)
  self.assertEqual(self.request(server,'GET','/api/t2-test')[0],403)
  code,headers,body=self.request(server,'GET','/t2-test.html');self.assertEqual(code,200);self.assertIn('T2 单独实测',body.decode());self.assertIn('开始 T2 实测',body.decode());self.assertIn('到达T2后',body.decode());self.assertNotIn('原6、7、8',body.decode());self.assertIn('Set-Cookie',headers)
  headers=self.session(server)
  self.assertEqual(self.request(server,'GET','/api/t2-test',headers=headers)[0],200)
  code,_,body=self.request(server,'GET','/api/navigation-view',headers=headers);self.assertEqual(json.loads(body)['t2_test']['scope'],'t2_test')
  self.assertEqual(self.request(server,'POST','/api/t2-test',json.dumps(dict(action='start',request_id='one')),headers)[0],202)
  self.assertEqual(self.request(server,'POST','/api/t2-test','{}',dict(headers,Origin='http://other'))[0],403);self.assertEqual(task.action.call_count,1);self.assertEqual(adapter.calls,[])
if __name__=='__main__':unittest.main()
