import unittest
import numpy as np,cv2
from offline_replay import replay_frames, preview_shallow

def d435_frame():
    h,w=300,500; yy,xx=np.mgrid[:h,:w]
    xyz=np.stack([(xx-250)*.001,(yy-150)*.001,np.ones((h,w))],-1)
    rgb=np.full((h,w,3),150,np.uint8);cv2.rectangle(rgb,(155,100),(345,199),(255,0,0),10)
    return dict(rgb=rgb,xyz=xyz,base_from_color=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,1.5],[0,0,0,1.]]),id="synthetic435",camera_id="d435",image_time=100.,arm_time=100.,body_time=100.,calibration_sha256="a"*64)

def d405_frame():
    rgb=np.full((480,640,3),230,np.uint8);cv2.rectangle(rgb,(200,60),(440,150),(255,0,0),-1);cv2.rectangle(rgb,(220,80),(420,145),(230,230,230),-1);cv2.rectangle(rgb,(190,100),(370,460),(25,25,25),-1)
    xx,yy=np.meshgrid(np.arange(640),np.arange(480));z=np.full_like(xx,.2,dtype=float);xyz=np.stack(((xx-320)*z/400,(yy-240)*z/400,z),axis=2);T=np.eye(4);T[:3,:3]=[[0,0,1],[0,-1,0],[1,0,0]]
    return dict(camera_id="d405",rgb=rgb,xyz=xyz,base_from_color=T)

class ReplayTest(unittest.TestCase):
    def test_detector_pipeline_outputs_nonexecuting_measured_results(self):
        result=replay_frames(d435_frame(),d405_frame())
        self.assertEqual(result["cabinet"]["camera"],"d435")
        self.assertEqual(result["vertical"]["camera"],"d405")
        self.assertGreater(result["vertical"]["raw_error_mm"],0)
        self.assertFalse(result["executable"])
    def test_failed_detection_does_not_invent_target(self):
        frame=d435_frame();frame["rgb"][:]=150
        with self.assertRaises(ValueError):replay_frames(frame,d405_frame())

    def test_shallow_plan_remains_horizontal_and_not_executable(self):
        import json
        from pathlib import Path
        from agent.cabinet_preview_geometry import localize,opening
        root=Path("/home/arx/arx-place-hold-20261116")
        frame=d435_frame();window=localize(frame,now=100.)
        slot=opening(json.loads((root/"config/cabinet-profiles.json").read_text()),"demo")
        battery=json.loads((root/"config/battery-envelope.json").read_text())
        tip=json.loads((root/"web/public/cad/battery-grip.json").read_text())["points_object_mm"]["tip"]
        plan=preview_shallow(window,slot,battery,tip,right_offset_mm=0.)
        xyz=np.asarray(plan["tcp_points_base_mm"])
        self.assertLess(np.ptp(xyz[:,2]),1e-8)
        self.assertAlmostEqual(plan["leading_insertion_depth_mm"],5.,delta=.01)
        self.assertFalse(plan["executable"])
        skew=d435_frame(); angle=np.radians(1.3); rot=np.eye(4)
        rot[:3,:3]=[[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]]
        skew["base_from_color"]=rot@skew["base_from_color"]
        skew_window=localize(skew,now=100.)
        skew_plan=preview_shallow(skew_window,slot,battery,tip,right_offset_mm=0.)
        self.assertAlmostEqual(skew_plan["leading_insertion_depth_mm"],5.,delta=.02)
        shifted=preview_shallow(window,slot,battery,tip,right_offset_mm=8.)
        delta=np.asarray(shifted["tcp_points_base_mm"])-xyz
        np.testing.assert_allclose(delta, np.tile(-8.*np.asarray(window["left_axis_base"]),(len(xyz),1)),atol=1e-8)
        with self.assertRaises(ValueError):preview_shallow(window,slot,battery,tip,right_offset_mm=8.1)
        with self.assertRaises(ValueError):preview_shallow(window,slot,battery,tip,right_offset_mm=-.1)
if __name__=="__main__":unittest.main()
