import unittest,threading,tempfile
from pathlib import Path
from http.server import ThreadingHTTPServer
import test_cabinet_http as http_fixture
from server import make_handler
from test_control import Controller,FakeAdapter,configured
from cabinet_execution import CabinetExecutionTask

class ExecutionHTTPTests(unittest.TestCase):
 request=http_fixture.CabinetHTTPTests.request
 session=http_fixture.CabinetHTTPTests.session
 tearDown=http_fixture.CabinetHTTPTests.tearDown
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  self.adapter=FakeAdapter();self.ctrl=Controller(configured(),self.adapter);self.cab=http_fixture.Cabinet()
  def blocked(model):raise ValueError('Current loaded sweep intersects aperture')
  self.task=CabinetExecutionTask(self.ctrl,None,directory=self.tmp.name,planner=blocked)
  self.server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.ctrl,cabinet_preview=self.cab,cabinet_execution=self.task,cabinet_rehearsal=object()))
  self.server.daemon_threads=True;self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start();self.port=self.server.server_port
 def test_real_task_route_auth_and_no_browser_override(self):
  path='/api/cabinet-execution';d={'op':'start','model':'demo'}
  self.assertEqual(self.request('GET',path)[0],403);self.assertEqual(self.request('POST',path,d)[0],403)
  h=self.session();self.assertEqual(self.request('POST',path,d,dict(h,Origin='http://wrong.local'))[0],403)
  self.assertEqual(self.request('POST',path,dict(d,target=[0]*6),h)[0],400)
  code,_,raw=self.request('POST',path,d,h)
  import json
  self.assertEqual(code,200);self.assertEqual(json.loads(raw)['status'],'blocked');self.assertEqual(self.adapter.calls,[])
  self.assertEqual(self.request('POST',path,{'op':'cancel'},h)[0],200)
