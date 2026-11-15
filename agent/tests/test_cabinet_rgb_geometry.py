import json
import time
import unittest
import cv2
import numpy as np
from agent import cabinet_rgb_geometry as g


def rgb_frame():
    now = time.time()
    rgb = np.full((480, 640, 3), 110, np.uint8)
    cv2.rectangle(rgb, (245, 205), (394, 279), (255, 0, 0), 8)
    return dict(rgb=rgb, id='rgb-only', camera_id='d435', camera_serial='349622074226',
                image_time=now, arm_time=now, body_time=now,
                calibration_sha256='a'*64,
                base_from_color=np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 1.5], [0, 0, 0, 1.]]),
                intrinsics=json.dumps(dict(width=640, height=480, fx=650., fy=650., cx=320., cy=240.,
                    model='distortion.inverse_brown_conrady', coeffs=[0.]*5, serial='349622074226')))


class RGBGeometryTests(unittest.TestCase):
    def test_rgb_only_frame_produces_camera_pose_and_base_axes(self):
        f=rgb_frame();r=g.localize_rgb(f, {'size_mm':[169.6,79.]})
        self.assertEqual(r['localization_source'],'rgb_cad_pnp')
        self.assertFalse(r['executable']);self.assertGreater(r['inside_axis_base'][0], .95)
        self.assertTrue(.6 < r['center_base_m'][0] < 1.)
        self.assertGreaterEqual(len(r['pose_candidates']),1)
        self.assertNotIn('rim_plane_p90_mm',r)
        self.assertEqual(r['model_aperture_size_mm'],[169.6,79.])

    def test_exact_distorted_projection_recovers_known_pose(self):
        k=json.loads(rgb_frame()['intrinsics']);k.update(model='distortion.brown_conrady',coeffs=[.1,-.03,.001,.002,0.])
        obj=np.array([[-.0848,.0395,0],[.0848,.0395,0],[.0848,-.0395,0],[-.0848,-.0395,0]])
        rv=np.array([3.02,.1,.2]);translation=np.array([.04,-.02,.7])
        K=np.array([[650.,0,320.],[0,650.,240.],[0,0,1.]])
        pixels=cv2.projectPoints(obj,rv,translation,K,np.array(k['coeffs']))[0].reshape(4,2)
        r=g.solve_quad_pose(pixels,k,[169.6,79.])[0]
        np.testing.assert_allclose(np.array(r['T_color_from_opening'])[:3,3],translation,atol=1e-6)
        self.assertLess(r['reprojection_rms_px'],1e-6)

    def test_unsupported_nonzero_distortion_is_rejected(self):
        f=rgb_frame();k=json.loads(f['intrinsics']);k['coeffs'][0]=.1;f['intrinsics']=json.dumps(k)
        with self.assertRaisesRegex(ValueError,'畸变'):g.localize_rgb(f,{'size_mm':[169.6,79.]})

    def test_bad_intrinsics_or_serial_cannot_create_metric_pose(self):
        for change in [dict(fx=0),dict(width=123),dict(serial='another-camera'),dict(fy=float('nan'))]:
            with self.subTest(change=change):
                f=rgb_frame();k=json.loads(f['intrinsics']);k.update(change);f['intrinsics']=json.dumps(k)
                with self.assertRaises(ValueError):g.localize_rgb(f,{'size_mm':[169.6,79.]})

    def test_stale_unpaired_or_unidentified_frames_are_rejected(self):
        for change in [dict(image_time=time.time()-5),dict(arm_time=time.time()-1),dict(id=''),dict(calibration_sha256='bad')]:
            f=rgb_frame();f.update(change)
            with self.subTest(change=change),self.assertRaises(ValueError):g.localize_rgb(f,{'size_mm':[169.6,79.]})

    def test_partial_and_multiple_targets_do_not_create_metric_pose(self):
        for multiple in [False,True]:
            f=rgb_frame()
            if multiple:cv2.rectangle(f['rgb'],(30,30),(160,95),(255,0,0),8)
            else:f['rgb']=np.roll(f['rgb'],-280,axis=1)
            with self.assertRaises(ValueError):g.localize_rgb(f,{'size_mm':[169.6,79.]})
