import http.client
import json
from pathlib import Path
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from control import Controller
from server import make_handler
from placement_task import PlacementTask
from test_control import FakeAdapter,configured

class Preview:
    def __init__(self):self.latest={};self.calls=[]
    def action(self,data):self.calls.append(data);return dict(status='localized',side='left',executable=False)

class PlacementHTTPTests(unittest.TestCase):
    def setUp(self):
        cfg=configured();cfg['execution_enabled']=False
        self.adapter=FakeAdapter();self.controller=Controller(cfg,self.adapter);self.preview=Preview()
        self.server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.controller,placement_preview=self.preview));self.server.daemon_threads=True
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start();self.port=self.server.server_port
    def tearDown(self):self.server.shutdown();self.server.server_close();self.thread.join()
    def request(self,method,path,data=None,headers=None):
        conn=http.client.HTTPConnection('127.0.0.1',self.port);conn.request(method,path,None if data is None else json.dumps(data),headers or {});r=conn.getresponse();value=(r.status,dict(r.getheaders()),r.read());conn.close();return value
    def session(self):
        code,h,_=self.request('GET','/placement.html');self.assertEqual(code,200)
        return {'Cookie':h['Set-Cookie'].split(';')[0],'Origin':'http://127.0.0.1:'+str(self.port),'Content-Type':'application/json'}
    def test_opening_page_and_reading_state_never_captures_or_moves(self):
        h=self.session();code,_,raw=self.request('GET','/api/placement-task',headers=h)
        self.assertEqual(code,200);self.assertFalse(json.loads(raw)['execution_enabled']);self.assertEqual(self.preview.calls,[]);self.assertEqual(self.adapter.calls,[])
    def test_preview_requires_session_and_same_origin(self):
        self.assertEqual(self.request('POST','/api/placement-preview',{'camera':'d435'})[0],403)
        h=self.session();self.assertEqual(self.request('POST','/api/placement-preview',{'camera':'d435'},h)[0],200)
        self.assertEqual(self.preview.calls,[{'camera':'d435'}]);self.assertEqual(self.adapter.calls,[])
    def test_disabled_execution_does_not_reserve_or_submit(self):
        h=self.session();code,_,_=self.request('POST','/api/placement-task',{'action':'start'},h)
        self.assertEqual(code,503);self.assertIsNone(self.controller.motion_owner);self.assertEqual(self.adapter.calls,[])
    def test_preview_get_also_requires_cookie(self):
        self.assertEqual(self.request('GET','/api/placement-preview')[0],403)
    def test_holding_confirmation_is_required_before_reservation(self):
        task=PlacementTask('/not-used',self.controller,lambda *a:None,execution_enabled=True)
        with self.assertRaises(ValueError):task.action('client',{'action':'start'})
        self.assertIsNone(self.controller.motion_owner)

if __name__=='__main__':unittest.main()

class PlacementRecoveryTests(unittest.TestCase):
    def test_restart_blocks_inflight_release_before_reserving(self):
        import tempfile
        from agent.storage import RunStore
        with tempfile.TemporaryDirectory() as root:
            s=RunStore.create(root);s.state['placement_flow']={'status':'running','phase':'release','inflight':True,'motion_started':True};s.save();s.close()
            ctrl=Controller(configured(),FakeAdapter());task=PlacementTask(root,ctrl,lambda *a:None,execution_enabled=True)
            self.assertTrue(task.snapshot('client')['recovery_required'])
            with self.assertRaises(ValueError):task.action('client',dict(action='start',request_id='00000000-0000-4000-8000-000000000001',expected_epoch=0,environment_clear=True,holding_confirmed=True))
            self.assertIsNone(ctrl.motion_owner)
    def test_old_ambiguous_attempt_blocks_same_process_new_run(self):
        import tempfile
        from agent.storage import RunStore
        with tempfile.TemporaryDirectory() as root:
            ctrl=Controller(configured(),FakeAdapter());task=PlacementTask(root,ctrl,lambda *a:None,execution_enabled=True)
            s=RunStore.create(root);s.state['placement_flow']={'status':'needs_attention','phase':'release','inflight':True,'motion_started':True};s.save();s.close()
            self.assertTrue(task.snapshot('client')['recovery_required'])
            with self.assertRaises(ValueError):task._start('client',0,True,{})
            self.assertIsNone(ctrl.motion_owner)
