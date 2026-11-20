"""Local navigation communication only. Never imports a hardware/task backend."""
import json,threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlsplit
from pathlib import Path
from task_handoff import ProtocolError,strict_json

class TaskServer:
 def __init__(self,hub,port=9985):
  class Handler(BaseHTTPRequestHandler):
   def log_message(self,*_):pass
   def reply(self,code,value):
    self.send_bytes(code,json.dumps(value,allow_nan=False).encode(),'application/json')
   def send_bytes(self,code,raw,kind):
    self.send_response(code)
    self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(raw)));self.send_header('Cache-Control','no-store')
    self.send_header('X-Content-Type-Options','nosniff');self.send_header('Referrer-Policy','no-referrer')
    self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
    self.end_headers();self.wfile.write(raw)
   def safe_request(self,write=False):
    host=self.headers.get('Host','')
    try:
     if urlsplit('http://'+host).hostname not in ('127.0.0.1','localhost'):raise ValueError('host')
     origin=self.headers.get('Origin')
     if write and origin is not None and origin!='http://'+host:raise ValueError('origin')
     if write and self.headers.get('Sec-Fetch-Site')=='cross-site':raise ValueError('cross-site')
    except ValueError:
     self.reply(403,dict(error='local same-origin request required'));return False
    return True
   def token(self):
    header=self.headers.get('Authorization','')
    if not header.startswith('Bearer '):raise PermissionError('worker token required')
    return header[7:]
   def do_GET(self):
    if not self.safe_request():return
    path=urlsplit(self.path).path
    try:
     assets={'/':('task_console.html','text/html; charset=utf-8'),'/console.mjs':('task_console.mjs','text/javascript; charset=utf-8'),'/console.css':('task_console.css','text/css; charset=utf-8')}
     if path in assets:
      name,kind=assets[path]
      try:raw=Path(__file__).with_name(name).read_bytes()
      except OSError:return self.reply(503,dict(error='console assets unavailable'))
      return self.send_bytes(200,raw,kind)
     if path=='/v1/state':return self.reply(200,hub.state())
     if path=='/v1/task':return self.reply(200,dict(task=hub.poll(self.token())))
     self.reply(404,dict(error='unknown_endpoint'))
    except PermissionError as e:self.reply(403,dict(error=str(e)))
    except ProtocolError as e:self.reply(409,dict(error=str(e)))
   def do_POST(self):
    if not self.safe_request(write=True):return
    path=urlsplit(self.path).path
    if path not in ('/v1/worker','/v1/heartbeat','/v1/task'):return self.reply(404,dict(error='unknown_endpoint'))
    try:
     if self.headers.get('Content-Type','').split(';')[0]!='application/json':return self.reply(415,dict(error='JSON required'))
     length=int(self.headers.get('Content-Length','0'))
     if not 0<length<=32768:return self.reply(413,dict(error='invalid request size'))
     try:value=strict_json(self.rfile.read(length))
     except ProtocolError as e:return self.reply(400,dict(error=str(e)))
     if path=='/v1/worker':result=hub.register(value)
     elif path=='/v1/heartbeat':
      if value!={}:raise ProtocolError('heartbeat accepts empty object')
      result=hub.heartbeat(self.token())
     else:result=hub.event(self.token(),value)
     self.reply(200,result)
    except PermissionError as e:self.reply(403,dict(error=str(e)))
    except (ProtocolError,ValueError,TypeError) as e:self.reply(409,dict(error=str(e)))
  self.http=ThreadingHTTPServer(('127.0.0.1',int(port)),Handler);self.http.daemon_threads=True;self.port=self.http.server_port
  self.thread=threading.Thread(target=self.http.serve_forever,daemon=True,name='navigation-task-http');self.thread.start()
 def close(self):self.http.shutdown();self.http.server_close();self.thread.join(timeout=2.)
