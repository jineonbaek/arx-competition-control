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
