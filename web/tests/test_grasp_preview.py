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
                self.assertNotIn('left_joints',f.files)

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


class HeadCaptureTests(unittest.TestCase):
    def test_head_uses_fixed_cad_transform_not_wrist_or_lift(self):
        import cv2, json
        import grasp_preview as gp
        T=np.eye(4);T[:3,3]=[.1,.2,1.2]
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);cal=root/'head.json'
            cal.write_text(json.dumps(dict(serial='head',T_base_from_color_optical=T.tolist())))
            preview=GraspPreview(Mock(),Mock(),cache=root,camera='d435')
            k=dict(width=2,height=2,serial='head')
            meta=dict(color_intrinsics=k,depth=dict(scale_m_per_unit=.001),captured_at=5.)
            meta['paired_left_feedback']=dict(timestamp=4.95,joints=[.1]*6)
            jpg=cv2.imencode('.jpg',np.zeros((2,2,3),np.uint8))[1].tobytes()
            png=cv2.imencode('.png',np.ones((2,2),np.uint16))[1].tobytes()
            with patch.object(gp,'HEAD_CALIBRATION',cal),patch.object(preview,'read_pair',
                return_value=(meta,jpg,png,np.ones(6),5.,15.,5.)),patch.object(gp,'xyz_image',
                return_value=np.ones((2,2,3),np.float32)),patch.object(gp,'ColumnModel') as model:
                view=preview.capture()
                model.assert_not_called()
                np.testing.assert_allclose(view['base_from_color'],T)
                with np.load(root/'frame.npz') as f:
                    self.assertEqual(str(f['camera_id']),'d435')
                    np.testing.assert_allclose(f['left_joints'],[.1]*6)
                    self.assertEqual(float(f['left_arm_time']),4.95)
                    self.assertEqual(str(f['calibration_sha256']),__import__('hashlib').sha256(cal.read_bytes()).hexdigest())
            preview.depth.acquire_grasp.assert_called_once_with('d435')
            preview.ctrl.submit.assert_not_called()

    def test_head_range_is_explicit_and_d405_default_unchanged(self):
        k=dict(width=2,height=2,fx=10,fy=10,cx=0,cy=0,model='distortion.none',coeffs=[0]*5)
        raw=np.full((2,2),2000,np.uint16)
        self.assertFalse(xyz_image(raw,.001,k).any())
        self.assertAlmostEqual(float(xyz_image(raw,.001,k,max_depth_m=3.)[0,0,2]),2.)

class LeftPairTests(unittest.TestCase):
    def test_requires_distinct_stable_paired_feedback(self):
        from grasp_preview import paired_left
        rows=[dict(timestamp=10+i*.1,joints=[0.]*6,fresh=True) for i in range(5)]
        result=paired_left(rows,10.3)
        self.assertEqual(result['timestamp'],10.3)
        for bad in ([rows[0]]*5,[None]*5,rows[:2]):self.assertIsNone(paired_left(bad,10.3))
        self.assertIsNone(paired_left(rows,12))
        rows[-1]['joints'][0]=.1
        self.assertIsNone(paired_left(rows,10.3))
