import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from server import make_handler
from camera_depth_control import DepthControl
from control import Controller
from test_control import FakeAdapter,configured

class DepthHTTPTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();root=Path(self.temp.name)
        self.depth=DepthControl(root/'requests.json',root);self.session=self.depth.reset()
        self.frame(False)
        self.adapter=FakeAdapter();self.controller=Controller(configured(),self.adapter)
        self.server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.controller,depth_control=self.depth))
        self.server.daemon_threads=True;self.server.stopping=threading.Event()
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.port=self.server.server_port
        _,headers,_=self.request('GET','/')
        self.headers={'Content-Type':'application/json','Cookie':headers['Set-Cookie'].split(';')[0],'Origin':f'http://127.0.0.1:{self.port}'}
    def tearDown(self):
        self.server.stopping.set();self.server.shutdown();self.server.server_close();self.thread.join();self.temp.cleanup()
    def frame(self,enabled):
        data=dict(state='online',capture_session=self.session,updated_at=time.time(),depth=dict(requested=enabled,state='online' if enabled else 'disabled'))
        (self.depth.frames/'d405.json').write_text(json.dumps(data))
    def request(self,method,path,body=None,headers=None):
        c=http.client.HTTPConnection('127.0.0.1',self.port,timeout=3);c.request(method,path,body,headers or {})
        r=c.getresponse();result=(r.status,dict(r.getheaders()),r.read());c.close();return result
    def test_request_is_not_reported_as_hardware_confirmation(self):
        self.assertEqual(self.request('GET','/api/camera/depth')[0],200)
        code,_,body=self.request('POST','/api/camera/depth',json.dumps(dict(camera='d405',enabled=True)),self.headers)
        self.assertEqual(code,202);status=json.loads(body)['cameras']['d405']
        self.assertEqual(status['state'],'starting');self.assertFalse(status['active'])
        self.frame(True)
        self.assertTrue(json.loads(self.request('GET','/api/camera/depth')[2])['cameras']['d405']['active'])
        code,_,body=self.request('POST','/api/camera/depth',json.dumps(dict(camera='d405',enabled=False)),self.headers)
        self.assertEqual(code,202);self.assertEqual(json.loads(body)['cameras']['d405']['state'],'stopping')
        self.frame(False)
        self.assertEqual(json.loads(self.request('GET','/api/camera/depth')[2])['cameras']['d405']['state'],'disabled')
        self.assertEqual(self.adapter.calls,[])
    def test_invalid_origin_cookie_payload_and_duplicate_keys_cannot_start_depth(self):
        body=json.dumps(dict(camera='d405',enabled=True))
        for headers in [{**self.headers,'Origin':'http://evil.invalid'},{k:v for k,v in self.headers.items() if k!='Cookie'}]:
            self.assertEqual(self.request('POST','/api/camera/depth',body,headers)[0],403)
        for body in ['{','{"camera":"d405","enabled":true,"enabled":false}',json.dumps(dict(camera='other',enabled=True)),json.dumps(dict(camera='d405',enabled=1))]:
            self.assertEqual(self.request('POST','/api/camera/depth',body,self.headers)[0],400)
        self.assertFalse(self.depth.enabled('d405'));self.assertEqual(self.adapter.calls,[])

if __name__=='__main__':unittest.main()
