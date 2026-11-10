import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from handle_preview import handle_line


class HandleLineTests(unittest.TestCase):
    def setUp(self):
        y,z=np.meshgrid(np.linspace(-.02,.02,48),np.linspace(-.01,.01,24))
        self.xyz=np.stack([y,z,np.ones_like(y)*.3],axis=-1)
        self.t=np.eye(4)
        self.t[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        self.t[:3,3]=[.2,-.25,.7]
        self.roi=[0,0,48,24]

    def test_center_hole_and_some_background(self):
        self.xyz[9:15,17:31]=0
        self.xyz[:2,:,2]=.8
        result=handle_line(self.xyz,self.t,self.roi,50)
        np.testing.assert_allclose(result['feature_mm'],[500,-250,700],atol=1)
        np.testing.assert_allclose(np.subtract(result['feature_mm'],result['pre_mm']),[50,0,0])

    def test_ambiguous_or_sparse_depth(self):
        self.xyz[:12,:,2]=.8
        with self.assertRaises(ValueError):handle_line(self.xyz,self.t,self.roi,50)
        self.xyz[:]=0
        with self.assertRaises(ValueError):handle_line(self.xyz,self.t,self.roi,50)

    def test_teaching_uses_stable_measured_wrist_on_same_frozen_frame(self):
        import json, tempfile, time
        from unittest.mock import Mock,patch
        import grasp_preview as gp
        now=time.time()
        with tempfile.TemporaryDirectory() as tmp:
            cache=Path(tmp)/"preview"
            cache.mkdir()
            np.savez(cache/"frame.npz",id="demo",xyz=self.xyz,base_from_color=self.t,
                     lift_rad=10.,image_time=now,intrinsics="{}")
            ctrl=Mock()
            pose=[399.,-249.,706.,90.,0.,0.]
            ctrl.snapshot.side_effect=[dict(chassis_active=False,body={"height":10.},
                arms={"right":dict(fresh=True,base_wrist=pose,joints=[0]*6,timestamp=now+n*.08)})
                for n in range(6)]
            preview=gp.GraspPreview(ctrl,Mock())
            req=dict(op="teach",confirmed=True,id="demo",roi=self.roi,clearance_mm=50)
            with patch.object(gp,"CACHE",cache),patch.object(gp,"camera_intrinsics",return_value=(Mock(),Mock())),patch.object(gp.time,"sleep"):
                with self.assertRaises(ValueError):preview.handle_action(dict(req,id="old"))
                with self.assertRaises(ValueError):preview.handle_action(dict(req,confirmed=False))
                result=preview.handle_action(req)
                np.testing.assert_allclose(result["wrist_end_mm"],pose[:3])
                np.testing.assert_allclose(result["wrist_pre_mm"],[349.,-249.,706.])
                self.assertEqual(preview.handle_action(dict(req,op="plan"))["teach"]["snapshot"],"demo")
                ctrl.submit.assert_not_called()
                preview.depth.acquire_grasp.assert_not_called()
