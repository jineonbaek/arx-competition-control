import http.client,json,threading,unittest
from http.server import ThreadingHTTPServer
from server import make_handler
from control import Controller
from test_control import FakeAdapter,configured

class Stages:
    def __init__(self):self.calls=[]
    def snapshot(self):return dict(status='idle',settings={'cabinet_z_mm':630.})
    def action(self,data):self.calls.append(data);return self.snapshot()
    def close(self):pass

class StageHTTPTests(unittest.TestCase):
    def setUp(self):
        cfg=configured();cfg['execution_enabled']=False;cfg['chassis_backend']='local'
        self.adapter=FakeAdapter();self.ctrl=Controller(cfg,self.adapter);self.stages=Stages()
        self.server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.ctrl,placement_stages=self.stages))
        self.server.daemon_threads=True;self.server.stopping=threading.Event()
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start();self.port=self.server.server_port
    def tearDown(self):
        self.server.stopping.set();self.server.shutdown();self.server.server_close();self.thread.join()
    def request(self,method,path,data=None,headers=None):
        c=http.client.HTTPConnection('127.0.0.1',self.port,timeout=3);c.request(method,path,None if data is None else data,headers or {})
        r=c.getresponse();value=(r.status,dict(r.getheaders()),r.read());c.close();return value
    def test_get_does_not_capture_and_post_requires_session_origin_and_strict_json(self):
        self.assertEqual(self.request('GET','/api/placement-stages')[0],403)
        _,h,_=self.request('GET','/visual-docking.html')
        headers={'Cookie':h['Set-Cookie'].split(';')[0],'Origin':f'http://127.0.0.1:{self.port}','Content-Type':'application/json'}
        code,_,body=self.request('GET','/api/placement-stages',headers=headers)
        self.assertEqual(code,200);self.assertEqual(json.loads(body)['settings']['cabinet_z_mm'],630.)
        self.assertEqual(self.stages.calls,[]);self.assertEqual(self.adapter.calls,[])
        for headers_bad in ({**headers,'Origin':'http://evil.invalid'},{k:v for k,v in headers.items() if k!='Cookie'}):
            self.assertEqual(self.request('POST','/api/placement-stages','{"op":"record_preobserve"}',headers_bad)[0],403)
        self.assertEqual(self.request('POST','/api/placement-stages','{"op":"observe_on","op":"run"}',headers)[0],400)
        self.assertEqual(self.stages.calls,[])
        self.assertEqual(self.request('POST','/api/placement-stages','{"op":"record_preobserve"}',headers)[0],202)
        self.assertEqual(self.stages.calls,[{'op':'record_preobserve'}]);self.assertEqual(self.adapter.calls,[])

if __name__=='__main__':unittest.main()
