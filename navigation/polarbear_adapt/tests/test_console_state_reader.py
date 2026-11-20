import unittest,json,threading,sys,os
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.request import Request
from pathlib import Path
sys.path.insert(0,os.getenv('ARX_EDIT_MODULE_DIR',str(Path(__file__).resolve().parents[1])))
from chassis_command import WebChassis
class ConsoleRead(unittest.TestCase):
 def test_console_state_is_readable_without_creating_ros_or_sending_output(self):
  calls=[]
  class Handler(BaseHTTPRequestHandler):
   def log_message(self,*args):pass
   def do_GET(self):
    calls.append(self.path);self.send_response(200)
    if self.path=='/':self.send_header('Set-Cookie','session=local; Path=/')
    self.end_headers();self.wfile.write(json.dumps({'execution_enabled':True,'body':{'fresh':True},'cookie':self.headers.get('Cookie')}).encode())
  with ThreadingHTTPServer(('127.0.0.1',0),Handler) as server:
   worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
   transport=WebChassis();transport.origin='http://127.0.0.1:'+str(server.server_port)
   try:
    with transport.client.open(transport.origin+'/api/state',timeout=.3) as r:state=json.load(r)
    self.assertTrue(state['body']['fresh']);self.assertEqual(state['cookie'],'session=local');self.assertIsNone(transport.ros_session)
    self.assertEqual(calls,['/','/api/state'])
    with self.assertRaises(ValueError):transport.client.open(Request(transport.origin+'/api/command',data=b'{}'))
    self.assertEqual(calls,['/','/api/state'])
   finally:server.shutdown();worker.join()
if __name__=='__main__':unittest.main()
