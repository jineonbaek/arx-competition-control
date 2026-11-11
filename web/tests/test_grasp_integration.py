import io,json,tempfile,time,unittest,uuid
from pathlib import Path
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from grasp_integration import GraspIntegration
from agent.config import Config
from agent.grasp_scene import save_frame
from agent.tests.test_grasp_integration import frame,pose
from agent.grasp_geometry import ROOT
import numpy as np
from PIL import Image

class PreviewChatTests(unittest.TestCase):
    def test_complete_preview_uses_existing_receipts_and_is_owner_scoped(self):
        with tempfile.TemporaryDirectory() as d:
            chat=GraspIntegration(Config(),Path(d))
            def capture(tools):
                buf=io.BytesIO();np.savez(buf,**frame())
                return save_frame(tools.store,buf.getvalue(),source="fixture")
            def recognize(store,ident,out,cancelled):
                Image.fromarray(np.full((20,30),255,np.uint8)).save(out/"mask.png")
                return {"status":"perception_accepted","bbox_xyxy":[0,0,30,20]}
            p=pose()
            import hashlib
            p["mesh_sha256"]=hashlib.sha256((ROOT/"web/cad/gripper-v2/battery-print.stl").read_bytes()).hexdigest()
            client=str(uuid.uuid4());state=chat.snapshot(client)
            payload={"action":"send","conversation_id":state["conversation_id"],"request_id":str(uuid.uuid4()),"message":"/preview 50"}
            with patch("agent.grasp_tools.grasp_scene.capture",side_effect=capture) as cap,patch("agent.grasp_pipeline.segment",side_effect=recognize),patch("agent.grasp_pipeline.estimate",return_value=p):
                chat.handle(client,payload);chat.handle(client,payload)
                end=time.monotonic()+5
                while chat.snapshot(client)["busy"] and time.monotonic()<end:time.sleep(.01)
            result=chat.result(client)
            self.assertEqual(result["status"],"preview_complete");self.assertFalse(result["executable"])
            self.assertIsNone(result["geometry"]["wrist_grip_mm"]);self.assertEqual(cap.call_count,1)
            with self.assertRaises(Exception):chat.result(str(uuid.uuid4()))
            state=chat.snapshot(client)
            chat.handle(client,{"action":"new","conversation_id":state["conversation_id"]})
            with self.assertRaises(Exception):chat.result(client)
            chat.close()

class PreviewHTTPTests(unittest.TestCase):
    def test_entry_session_origin_and_no_motion(self):
        import http.client,threading
        from http.server import ThreadingHTTPServer
        from unittest.mock import Mock
        from server import make_handler
        from control import Controller
        from test_control import FakeAdapter,configured
        adapter=FakeAdapter();preview=Mock()
        preview.snapshot.return_value={"busy":False}
        preview.handle.return_value={"busy":True}
        preview.result.return_value={"status":"preview_complete","executable":False}
        server=ThreadingHTTPServer(("127.0.0.1",0),make_handler(Controller(configured(),adapter),grasp_integration=preview))
        server.daemon_threads=True;server.stopping=threading.Event()
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        host=f"127.0.0.1:{server.server_port}"
        def req(method,path,body=None,headers=None):
            c=http.client.HTTPConnection(host,timeout=3);c.request(method,path,body,headers or {})
            r=c.getresponse();result=r.status,dict(r.getheaders()),r.read();c.close();return result
        try:
            self.assertEqual(req("GET","/api/grasp-integration")[0],403)
            code,headers,body=req("GET","/grasp-integration.html")
            self.assertEqual(code,200);self.assertIn(b"js/grasp-integration.js",body)
            good={"Cookie":headers["Set-Cookie"].split(";")[0],"Origin":"http://"+host,"Content-Type":"application/json"}
            self.assertEqual(req("GET","/api/grasp-integration",headers=good)[0],200)
            self.assertEqual(req("GET","/api/grasp-integration/result")[0],403)
            self.assertEqual(req("GET","/api/grasp-integration/result",headers=good)[0],200)
            self.assertEqual(req("POST","/api/grasp-integration",'{"action":"send"}',{**good,"Origin":"http://other"})[0],403)
            self.assertEqual(req("POST","/api/grasp-integration",'{"action":"send","action":"new"}',good)[0],400)
            preview.handle.assert_not_called()
            self.assertEqual(req("POST","/api/grasp-integration",'{"action":"send"}',good)[0],202)
            preview.handle.assert_called_once()
            self.assertIn(b'href="grasp-integration.html"',req("GET","/")[2])
            self.assertEqual(adapter.calls,[])
        finally:
            server.stopping.set();server.shutdown();server.server_close();thread.join()

if __name__=="__main__":unittest.main()
