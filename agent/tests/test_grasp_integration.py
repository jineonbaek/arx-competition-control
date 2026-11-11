import json, tempfile, unittest, sys
from unittest.mock import patch
from agent.grasp_scene import save_frame, frame_path
from agent.grasp_pipeline import pipeline, run_worker
from agent.contracts import AgentError
import io
from pathlib import Path
import numpy as np
from agent.grasp_geometry import preview_geometry, validate_frame
from agent.config import Config
from agent.storage import RunStore
from agent.grasp_tools import GraspTools

def frame():
    xyz=np.zeros((20,30,3));xyz[:]=[0,0,.4]
    return dict(id=np.array("a"*32),xyz=xyz,rgb=np.zeros((20,30,3),np.uint8),
      raw_depth=np.full((20,30),400,np.uint16),scale=np.array(.001),
      intrinsics=np.array(json.dumps(dict(width=30,height=20,fx=30,fy=30,cx=15,cy=10,serial="test"))),
      base_from_color=np.eye(4),image_time=np.array(10.),arm_time=np.array(10.05),body_time=np.array(10.05),lift_rad=np.array(3.))
def pose():
    return dict(id="a"*32,frame="base_link",T_camera_from_object=np.eye(4).tolist(),
        T_base_from_object=np.eye(4).tolist())
class GeometryTests(unittest.TestCase):
    def test_fixed_x_line_and_no_tcp_targets(self):
        out=preview_geometry(frame(),np.ones((20,30),bool),pose(),50)
        line=np.asarray(out["path"]["points_mm"])
        self.assertTrue(np.allclose(line[:,1:],line[0,1:]))
        self.assertAlmostEqual(line[-1,0]-line[0,0],50)
        self.assertIsNone(out["wrist_pregrasp_mm"]);self.assertIsNone(out["wrist_grip_mm"])
        self.assertFalse(out["executable"]);self.assertEqual(out["tcp_status"],"pending")
        self.assertFalse(out["whole_path_collision_checked"])
    def test_mismatched_pose_mask_and_transform(self):
        for mutate in ("id","transform","mask"):
            f,p,m=frame(),pose(),np.ones((20,30),bool)
            if mutate=="id":p["id"]="b"*32
            if mutate=="transform":p["T_base_from_object"][0][3]=1
            if mutate=="mask":m=np.ones((2,3),bool)
            with self.assertRaises(ValueError):preview_geometry(f,m,p,50)
    def test_invalid_pair_geometry_and_clearance(self):
        for key,value in [("base_from_color",np.zeros((4,4))),("body_time",np.array(9.)),("scale",np.array(float("nan")))]:
            f=frame();f[key]=value
            with self.assertRaises(ValueError):validate_frame(f)
        for distance in (0,-1,float("nan"),201):
            with self.assertRaises(ValueError):preview_geometry(frame(),np.ones((20,30),bool),pose(),distance)
    def test_no_physical_or_graspnet_tools(self):
        with tempfile.TemporaryDirectory() as d:
            s=RunStore.create(d)
            try:
                tools=GraspTools(Config(),s)
                names={x["function"]["name"] for x in tools.definitions()}
                self.assertNotIn("execute_pick",names);self.assertNotIn("generate_grasp_candidates",names)
                self.assertFalse(tools.execute("execute_pick",{"plan_id":"x"},"test")["ok"])
                with self.assertRaises(ValueError):GraspTools(Config(motion_enabled=True),s)
            finally:s.close()
class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.store=RunStore.create(self.temp.name);self.addCleanup(self.store.close)
        buf=io.BytesIO();np.savez(buf,**frame());self.blob=buf.getvalue()
        self.obs=save_frame(self.store,self.blob,source="test")
    def test_capture_uses_current_handle_api_and_checks_frame_identity(self):
        from agent.grasp_scene import capture
        from agent.tests.test_transport import Server,response
        root=Path(self.temp.name)/"owner"
        archive=root/"web/hardware/runtime/grasp-preview/frame.npz"
        archive.parent.mkdir(parents=True);archive.write_bytes(self.blob)
        cal=root/"cal.json";cal.write_text("{}")
        def respond(path,body):
            self.assertEqual(path,"/api/handle-preview")
            self.assertEqual(json.loads(body),{"op":"capture"})
            return response({"id":"a"*32})
        with Server(respond) as server,patch("agent.grasp_scene.ROOT",root),patch("agent.grasp_scene.CALIBRATION",cal):
            registry=GraspTools(Config(console_url=server.url),self.store)
            captured=capture(registry)
            self.assertEqual(captured["source_snapshot"],"a"*32)
            self.assertEqual(len(captured["calibration_sha256"]),64)
            other=frame();other["id"]=np.array("b"*32)
            buf=io.BytesIO();np.savez(buf,**other);archive.write_bytes(buf.getvalue())
            with self.assertRaises(AgentError) as error:capture(registry)
            self.assertEqual(error.exception.code,"stale_snapshot")
        self.assertTrue(all(method=="POST" for method,_,_ in server.requests))

    def test_archive_identity_and_tamper(self):
        with self.assertRaises(AgentError):save_frame(self.store,self.blob,source="test",expected_id="wrong")
        path=frame_path(self.store,self.obs["observation_id"])
        path.write_bytes(b"tampered")
        with self.assertRaises(AgentError):frame_path(self.store,self.obs["observation_id"])
    def test_rejected_recognition_does_not_run_pose(self):
        with patch("agent.grasp_pipeline.segment",return_value={"status":"needs_new_observation"}),patch("agent.grasp_pipeline.estimate") as estimate:
            result=pipeline(self.store,self.obs["observation_id"],50,self.store.path/"preview")
        estimate.assert_not_called();self.assertFalse(result["executable"])
        self.assertNotIn("geometry",result)
    def test_pose_depth_rejection_preserves_overlay_without_path(self):
        import hashlib
        from PIL import Image
        from agent.grasp_geometry import ROOT
        def segment_fixture(store,ident,output,cancelled):
            Image.fromarray(np.full((20,30),255,np.uint8)).save(output/'mask.png')
            return dict(status='perception_accepted',bbox_xyxy=[0,0,30,20])
        report=pose();report['mesh_sha256']=hashlib.sha256((ROOT/'web/cad/gripper-v2/battery-print.stl').read_bytes()).hexdigest()
        report['overlay']='diagnostic-image'
        for quality in (None,dict(version=1,status='rejected'),dict(version=1,status='insufficient_depth'),dict(version=1,status='consistent')):
            report['quality']=quality
            with patch('agent.grasp_pipeline.segment',side_effect=segment_fixture),patch('agent.grasp_pipeline.estimate',return_value=report):
                result=pipeline(self.store,self.obs['observation_id'],50,self.store.path/'quality')
            good=quality and quality['status']=='consistent'
            self.assertEqual(result['status'],'preview_complete' if good else 'pose_rejected')
            self.assertEqual('geometry' in result,bool(good))
            self.assertEqual(result['pose']['overlay'],'diagnostic-image')
            self.assertFalse(result['executable'])

    def test_worker_timeout_and_cancel_leave_no_child(self):
        with self.assertRaises(AgentError) as e:
            run_worker([sys.executable,"-c","import time; time.sleep(30)"],self.store.path,self.store.path/"timeout.log",lambda:False,.15)
        self.assertEqual(e.exception.code,"worker_timeout")
        with self.assertRaises(AgentError) as e:
            run_worker([sys.executable,"-c","raise Exception('must not launch')"],self.store.path,self.store.path/"cancel.log",lambda:True,1)
        self.assertEqual(e.exception.code,"cancelled")

if __name__=="__main__":unittest.main()
