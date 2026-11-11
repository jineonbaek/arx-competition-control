import sys
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from grasp_preview import GraspPreview,roi_pixels,xyz_image
from camera_depth_control import DepthControl
import tempfile


class PreviewTests(unittest.TestCase):
    def test_roi_matches_frozen_pixels(self):
        self.assertEqual(roi_pixels([5,12,40,50],60,60),[5,12,40,50])
        for invalid in ([0,0,100,10],[0,3,12,12],[0,False,25,25],None):
            with self.assertRaises(ValueError):roi_pixels(invalid,60,60)

    def test_deprojection_metres_and_invalid_zero(self):
        raw=np.array([[0,250],[500,5000]],dtype=np.uint16)
        k=dict(width=2,height=2,fx=10,fy=10,cx=0,cy=0,model='distortion.none',coeffs=[0]*5)
        xyz=xyz_image(raw,.001,k)
        np.testing.assert_allclose(xyz[0,1],[.025,0,.25],atol=1e-6)
        np.testing.assert_allclose(xyz[1,0],[0,.05,.5],atol=1e-6)
        self.assertFalse(xyz[0,0].any() or xyz[1,1].any())

    def test_depth_release_on_capture_error(self):
        depth=Mock();preview=GraspPreview(Mock(),depth);preview.model=Mock()
        with patch.object(preview,'read_pair',side_effect=ValueError('offline')):
            with self.assertRaises(ValueError):preview.capture()
        depth.release_handeye.assert_called_once_with(depth.acquire_grasp.return_value)
        preview.ctrl.submit.assert_not_called()

    def test_legacy_cache_requires_recapture(self):
        with tempfile.TemporaryDirectory() as tmp, patch("grasp_preview.CACHE", Path(tmp)):
            (Path(tmp)/"view.json").write_text('{"id":"old","result":{"body4_mm":[1,2,3]}}')
            self.assertIsNone(GraspPreview(Mock(),Mock()).latest()["id"])

    def test_capture_freezes_exposure_lift_in_base_transform(self):
        import cv2, json
        import grasp_preview as gp
        from handeye_column import ColumnModel, NOMINAL_RAD_PER_M
        preview=GraspPreview(Mock(),Mock())
        model=ColumnModel()
        preview.model=model
        q=np.array([.1,.2,.3,.2,.1,-.1])
        h=8.
        cal=json.loads(gp.CANDIDATE.read_text())
        k=dict(width=2,height=2,serial=cal["camera"]["serial"])
        meta=dict(color_intrinsics=k,depth=dict(scale_m_per_unit=.001),captured_at=5.)
        jpg=cv2.imencode(".jpg",np.zeros((2,2,3),np.uint8))[1].tobytes()
        png=cv2.imencode(".png",np.ones((2,2),np.uint16))[1].tobytes()
        with tempfile.TemporaryDirectory() as tmp, patch.object(gp,"CACHE",Path(tmp)), patch.object(
            preview,"read_pair",return_value=(meta,jpg,png,q,5.,h,5.)), patch.object(
            gp,"xyz_image",return_value=np.ones((2,2,3),np.float32)):
            view=preview.capture()
            self.assertEqual(view["frame_id"],"base_link")
            nominal=model.pose(q,0)[1] @ np.array(cal["T_right_link26_from_D405_color"])
            nominal[2,3]+=h/NOMINAL_RAD_PER_M
            np.testing.assert_allclose(view["points"][0][:3],nominal[:3,:3]@np.ones(3)+nominal[:3,3],atol=.0001)
            with np.load(Path(tmp)/"frame.npz") as f:
                np.testing.assert_allclose(f["base_from_color"],nominal,atol=1e-10)

    def test_handeye_preempts_grasp_and_restores_manual_setting(self):
        with tempfile.TemporaryDirectory() as d:
            control=DepthControl(Path(d)/'config.json',Path(d))
            control.reset()
            shot=control.acquire_grasp()
            arm=control.acquire_handeye(False)
            control.release_handeye(shot)
            self.assertFalse(control.enabled('d405'))
            control.release_handeye(arm)
            self.assertFalse(control.enabled('d405'))


    def test_removed_inference_cannot_acquire_depth_or_command_robot(self):
        preview=GraspPreview(Mock(),Mock())
        with tempfile.TemporaryDirectory() as tmp, patch("grasp_preview.CACHE", Path(tmp)):
            with self.assertRaisesRegex(ValueError,"未知预览操作"):
                preview.action({"op":"infer"})
        self.assertFalse(hasattr(preview,"infer"))
        preview.depth.acquire_grasp.assert_not_called()
        preview.ctrl.submit.assert_not_called()
