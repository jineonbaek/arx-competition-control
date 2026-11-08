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

class VisionHTTPTests(unittest.TestCase):
    def test_session_origin_and_camera_links(self):
        adapter=FakeAdapter()
        preview=Mock()
        preview.snapshot.return_value={"status":"idle","running":False}
        preview.action.return_value={"status":"running","running":True}
        server=ThreadingHTTPServer(("127.0.0.1",0),make_handler(Controller(configured(),adapter),vision=preview))
        server.daemon_threads=True;server.stopping=threading.Event()
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        host="127.0.0.1:"+str(server.server_port)
        def request(method,path,body=None,headers=None):
            client=http.client.HTTPConnection(host,timeout=3)
            client.request(method,path,body,headers or {})
            reply=client.getresponse()
            result=(reply.status,dict(reply.getheaders()),reply.read())
            client.close()
            return result
        try:
            self.assertEqual(request("GET","/api/vision")[0],403)
            status,headers,body=request("GET","/vision.html")
            self.assertEqual(status,200)
            cookie=headers["Set-Cookie"].split(";")[0]
            good={"Cookie":cookie,"Origin":"http://"+host,"Content-Type":"application/json"}
            self.assertEqual(request("GET","/api/vision",headers=good)[0],200)
            preview.action.assert_not_called()
            data=json.dumps({"op":"start","camera":"d435"})
            self.assertEqual(request("POST","/api/vision",data,{**good,"Origin":"http://evil.invalid"})[0],403)
            self.assertEqual(request("POST","/api/vision",data,good)[0],200)
            preview.action.assert_called_once()
            html=request("GET","/")[2].decode()
            self.assertIn("vision.html?camera=d435",html)
            self.assertIn("vision.html?camera=d405",html)
            self.assertNotIn("vision.html?camera=astra",html)
            self.assertEqual(adapter.calls,[])
        finally:
            server.stopping.set();server.shutdown();server.server_close();thread.join()
