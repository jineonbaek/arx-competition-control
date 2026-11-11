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
    def test_agent_selects_stage_tools_and_retains_owner_scoped_evidence(self):
        from grasp_integration import GraspRuntime
        from agent.tests.test_runtime import ScriptModel,call
        def answer(name,args,ident):
            return {'role':'assistant','content':'','tool_calls':[call(name,args,ident)]}
        with tempfile.TemporaryDirectory() as d:
            def factory(config,store,tools,**kwargs):
                def obs_id(camera):
                    return store.state['latest_battery_observations'][camera]
                model=ScriptModel([
                    answer('get_system_status',{},'status'),
                    answer('observe_battery',{'camera':'d435'},'head'),
                    lambda:answer('plan_coarse_alignment',{'observation_id':obs_id('d435')},'coarse'),
                    answer('observe_battery',{'camera':'d405'},'wrist'),
                    lambda:answer('preview_battery_grasp',{'observation_id':obs_id('d405'),'clearance_mm':50},'fine'),
                    {'role':'assistant','content':'预览完成，TCP 待提供；没有运动。'}])
                return GraspRuntime(config,store,tools,model=model,**kwargs)
            chat=GraspIntegration(Config(),Path(d),runtime_factory=factory)
            def capture(tools,camera='d405'):
                data=frame();data['camera_id']=np.array(camera)
                buf=io.BytesIO();np.savez(buf,**data)
                return save_frame(tools.store,buf.getvalue(),source='fixture',camera=camera)
            def recognize(store,ident,out,cancelled):
                Image.fromarray(np.full((20,30),255,np.uint8)).save(out/'mask.png')
                return {'status':'perception_accepted','bbox_xyxy':[0,0,30,20]}
            p=pose()
            import hashlib
            p['mesh_sha256']=hashlib.sha256((ROOT/'web/cad/gripper-v2/battery-print.stl').read_bytes()).hexdigest()
            client=str(uuid.uuid4());state=chat.snapshot(client)
            payload={'action':'send','conversation_id':state['conversation_id'],'request_id':str(uuid.uuid4()),'message':'从 D435 开始生成抓取流程预览，预抓 50 mm。'}
            with patch.dict('os.environ',{'QWEN_BASE_URL':'https://example.invalid/v1','DASHSCOPE_API_KEY':'fixture'}),patch(
                'agent.observations.ConsoleObservations.robot_state',return_value={'available':True}),patch(
                'agent.battery_observation.capture',side_effect=capture) as cap,patch(
                'agent.battery_observation.segment',side_effect=recognize) as rec,patch(
                'agent.grasp_pipeline.segment') as repeated,patch('agent.grasp_pipeline.estimate',return_value=p):
                chat.handle(client,payload);chat.handle(client,payload)
                end=time.monotonic()+5
                while chat.snapshot(client)['busy'] and time.monotonic()<end:time.sleep(.01)
            result=chat.result(client)
            self.assertEqual(result['status'],'preview_complete');self.assertFalse(result['executable'])
            self.assertIsNone(result['geometry']['wrist_grip_mm'])
            self.assertIsNone(result['coarse_alignment']['tcp'])
            self.assertEqual(set(result['observations']),{'d435','d405'})
            self.assertEqual(cap.call_count,2);self.assertEqual(rec.call_count,2);repeated.assert_not_called()
            run=Path(chat.sessions[client]['run_dir']);saved=json.loads((run/'state.json').read_text())
            names=[v['result']['tool'] for v in saved['calls'].values()]
            self.assertEqual(names,['get_system_status','observe_battery','plan_coarse_alignment','observe_battery','preview_battery_grasp'])
            self.assertFalse(saved['outcome']['mission_success'])
            with self.assertRaises(Exception):chat.result(str(uuid.uuid4()))
            state=chat.snapshot(client)
            chat.handle(client,{'action':'new','conversation_id':state['conversation_id']})
            with self.assertRaises(Exception):chat.result(client)
            chat.close()

    def test_cloud_credentials_required_and_no_hardcoded_preview_bypass(self):
        with tempfile.TemporaryDirectory() as d,patch.dict('os.environ',{},clear=True):
            chat=GraspIntegration(Config(),Path(d))
            client=str(uuid.uuid4());state=chat.snapshot(client)
            self.assertFalse(state['available'])
            with self.assertRaises(Exception):chat.handle(client,{'action':'send','conversation_id':state['conversation_id'],
                'request_id':str(uuid.uuid4()),'message':'/preview 50'})
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
