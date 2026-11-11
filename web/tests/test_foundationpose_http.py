import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from server import make_handler
from control import Controller
from test_control import FakeAdapter,configured


class FoundationPoseHTTPTests(unittest.TestCase):
    def test_session_origin_read_only_entry_and_strict_json(self):
        adapter=FakeAdapter();preview=Mock()
        preview.snapshot.return_value=dict(status="idle",running=False)
        preview.action.return_value=dict(status="running",running=True)
        server=ThreadingHTTPServer(("127.0.0.1",0),make_handler(Controller(configured(),adapter),foundation=preview))
        server.daemon_threads=True;server.stopping=threading.Event()
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        host=f"127.0.0.1:{server.server_port}"
        def req(method,path,body=None,headers=None):
            c=http.client.HTTPConnection(host,timeout=3);c.request(method,path,body,headers or {})
            r=c.getresponse();result=(r.status,dict(r.getheaders()),r.read());c.close();return result
        try:
            self.assertEqual(req("GET","/api/foundationpose")[0],403)
            status,headers,body=req("GET","/foundationpose.html")
            self.assertEqual(status,200);self.assertIn(b"js/foundationpose.js",body)
            good={"Cookie":headers["Set-Cookie"].split(";")[0],"Origin":"http://"+host,"Content-Type":"application/json"}
            self.assertEqual(req("GET","/api/foundationpose",headers=good)[0],200)
            preview.action.assert_not_called()
            self.assertEqual(req("POST","/api/foundationpose",'{"op":"capture"}',{**good,"Origin":"http://other"})[0],403)
            self.assertEqual(req("POST","/api/foundationpose",'{"op":"capture","op":"infer"}',good)[0],400)
            preview.action.assert_not_called()
            self.assertEqual(req("POST","/api/foundationpose",'{"op":"capture"}',good)[0],200)
            preview.action.assert_called_once()
            html=req("GET","/")[2].decode()
            start=html.index('data-camera="d405"');end=html.index("</article>",start)
            self.assertIn('href="foundationpose.html"',html[start:end])
            self.assertEqual(html.count('href="foundationpose.html"'),1)
            self.assertEqual(adapter.calls,[])
        finally:
            server.stopping.set();server.shutdown();server.server_close();thread.join()
