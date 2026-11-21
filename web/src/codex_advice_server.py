"""Narrow TLS mailbox for the Mac worker; no control or execution endpoints."""
import hmac
import json
import logging
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import ssl
import threading
from agent.codex_recovery import MAIL


def serve(directory=None,host='0.0.0.0',port=9994,mail=MAIL):
    folder=Path(directory or Path.home()/'.config/arx-codex-advice')
    if not (folder/'token').is_file():return None
    secret=(folder/'token').read_text().strip()
    if len(secret)!=64:raise ValueError('Invalid advice mailbox credential')
    context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(folder/'server.pem',folder/'server.key')
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def respond(self,code,value=None):
            data=json.dumps(value,separators=(',',':')).encode() if value is not None else b''
            self.send_response(code);self.send_header('Content-Type','application/json')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)))
            self.end_headers();self.wfile.write(data)
        def authorized(self):
            if hmac.compare_digest(self.headers.get('Authorization','').encode(),('Bearer '+secret).encode()):return True
            self.respond(403);return False
        def do_GET(self):
            if not self.authorized():return
            if self.path=='/health':
                mail.heartbeat()
                return self.respond(200,dict(service='codex-advice',version=2,**mail.snapshot()))
            if self.path!='/next':return self.respond(404)
            job=mail.take();self.respond(200 if job else 204,job)
        def do_POST(self):
            if not self.authorized():return
            if self.path=='/diagnostic-test':
                key=mail.diagnostic('connection-test','连接自检','只验证故障请求往返。请回复连接正常；不得建议或执行机器人运动。')
                return self.respond(200,dict(id=key,motion=False))
            if self.path!='/reply':return self.respond(404)
            try:
                n=int(self.headers.get('Content-Length','0'))
                if not 0<n<=4096:raise ValueError()
                msg=json.loads(self.rfile.read(n))
                if set(msg)!={'id','answer'} or not isinstance(msg['id'],str):raise ValueError()
                ok=mail.reply(msg['id'],msg['answer'])
            except (KeyError,ValueError,TypeError):return self.respond(400)
            self.respond(200 if ok else 409)
    class Server(ThreadingHTTPServer):
        daemon_threads=True
        def get_request(self):
            sock,addr=super().get_request();sock.settimeout(4.)
            # Handshake on the handler thread so one client cannot block accept.
            return context.wrap_socket(sock,server_side=True,do_handshake_on_connect=False),addr
    server=Server((host,port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True,name='codex-advice-mailbox').start()
    mail.enabled=True
    return server
