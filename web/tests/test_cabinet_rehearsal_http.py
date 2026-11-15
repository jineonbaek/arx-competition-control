import http.client,json,sys,threading,unittest,tempfile
from pathlib import Path
from http.server import ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from server import make_handler
from control import Controller
from test_control import FakeAdapter,configured
from cabinet_rehearsal import CabinetRehearsal
class RehearsalHTTPTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.a=FakeAdapter();self.c=Controller(configured(),self.a);self.r=CabinetRehearsal(self.c,Path(self.tmp.name));self.s=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.c,cabinet_rehearsal=self.r));self.s.daemon_threads=True;self.t=threading.Thread(target=self.s.serve_forever,daemon=True);self.t.start();self.port=self.s.server_port
 def tearDown(self):self.r.close();self.s.shutdown();self.s.server_close();self.t.join();self.tmp.cleanup()
 def request(self,method,path,data=None,headers=None):
  c=http.client.HTTPConnection('127.0.0.1',self.port);c.request(method,path,None if data is None else json.dumps(data),headers or {});r=c.getresponse();o=(r.status,dict(r.getheaders()),r.read());c.close();return o
 def headers(self):
  _,h,_=self.request('GET','/placement.html');return {'Cookie':h['Set-Cookie'].split(';')[0],'Origin':'http://127.0.0.1:'+str(self.port),'Content-Type':'application/json'}
 def test_status_and_default_disabled_start_never_move(self):
  h=self.headers();self.assertEqual(self.request('GET','/api/cabinet-rehearsal',headers=h)[0],200);self.assertEqual(self.request('POST','/api/cabinet-rehearsal',{'op':'start','run_id':'anything'},h)[0],400);self.assertEqual(self.a.calls,[])
 def test_origin_and_session_guard(self):
  self.assertEqual(self.request('POST','/api/cabinet-rehearsal',{'op':'start','run_id':'anything'})[0],403);self.assertEqual(self.request('GET','/api/cabinet-rehearsal')[0],403);self.assertEqual(self.a.calls,[])
 def test_cannot_supply_arbitrary_coordinates(self):
  h=self.headers();self.assertEqual(self.request('POST','/api/cabinet-rehearsal',{'op':'start','run_id':'anything','joints':[1]*6},h)[0],400);self.assertEqual(self.a.calls,[])
 def test_server_imports_with_only_script_directory_on_search_path(self):
  import os,subprocess,sys
  src=Path(__file__).resolve().parents[1]/'src';env=dict(os.environ,PYTHONPATH=str(src))
  result=subprocess.run([sys.executable,'-c',"import runpy;runpy.run_path("+repr(str(src/'server.py'))+",run_name='startup_import_check')"],cwd=self.tmp.name,env=env,capture_output=True,text=True)
  self.assertEqual(result.returncode,0,result.stderr)
 def test_source_manifest_covers_lift_and_column_geometry(self):
  from cabinet_rehearsal import default_sources
  sources=default_sources(Path(self.tmp.name));self.assertTrue({'base_link','body4'}<=sources.keys());self.assertTrue(sources['base_link'].is_file());self.assertTrue(sources['body4'].is_file())
