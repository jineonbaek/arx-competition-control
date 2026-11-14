import http.client,json,sys,threading,unittest
from pathlib import Path
from http.server import ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from server import make_handler
from control import Controller
from test_control import FakeAdapter,configured
class Cabinet:
    latest={}
    def __init__(self):self.calls=[]
    def action(self,data):
        if set(data)!={'camera','model'}:raise ValueError('invalid')
        self.calls.append(data);return {'status':'localized','executable':False,'plan':{'z_range_mm':0}}
class CabinetHTTPTests(unittest.TestCase):
    def setUp(self):
        self.adapter=FakeAdapter();self.ctrl=Controller(configured(),self.adapter);self.cab=Cabinet();self.server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.ctrl,cabinet_preview=self.cab));self.server.daemon_threads=True;self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start();self.port=self.server.server_port
    def tearDown(self):self.server.shutdown();self.server.server_close();self.thread.join()
    def request(self,method,path,data=None,headers=None):
        c=http.client.HTTPConnection('127.0.0.1',self.port);c.request(method,path,None if data is None else json.dumps(data),headers or {});r=c.getresponse();out=(r.status,dict(r.getheaders()),r.read());c.close();return out
    def session(self):
        _,h,_=self.request('GET','/placement.html');return {'Cookie':h['Set-Cookie'].split(';')[0],'Origin':'http://127.0.0.1:'+str(self.port),'Content-Type':'application/json'}
    def test_read_does_not_capture_or_move(self):
        h=self.session();self.assertEqual(self.request('GET','/api/cabinet-preview',headers=h)[0],200);self.assertEqual(self.cab.calls,[]);self.assertEqual(self.adapter.calls,[])
    def test_capture_only_has_no_robot_command(self):
        h=self.session();self.assertEqual(self.request('POST','/api/cabinet-preview',{'camera':'d435','model':'demo'},h)[0],200);self.assertEqual(self.adapter.calls,[])
    def test_cookie_origin_and_argument_guards(self):
        self.assertEqual(self.request('POST','/api/cabinet-preview',{'camera':'d435','model':'demo'})[0],403)
        h=self.session();self.assertEqual(self.request('POST','/api/cabinet-preview',{'camera':'d435','model':'demo','execute':True},h)[0],400);self.assertEqual(self.adapter.calls,[])
