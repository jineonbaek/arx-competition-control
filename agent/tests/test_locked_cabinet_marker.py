import unittest,numpy as np,json

class MarkerTests(unittest.TestCase):
    def test_static_marker_does_not_inherit_noisy_new_depth(self):
        from agent.locked_cabinet_marker import marker_drift_mm
        corners=np.array([[-.1,-.05,1.],[.1,-.05,1.],[.1,.05,1.],[-.1,.05,1.]])
        window=dict(center_base_m=[0,0,1],outward_normal_base=[0,0,-1],corners_base_m=corners.tolist(),calibration_sha256='a'*64)
        k=dict(width=640,height=480,fx=500.,fy=500.,cx=320.,cy=240.,coeffs=[0.]*5,model='distortion.none')
        frame=dict(rgb=np.zeros((480,640,3),np.uint8),intrinsics=json.dumps(k),base_from_color=np.eye(4),calibration_sha256='a'*64)
        quad=corners[:,:2]*500+[320,240]
        self.assertLess(marker_drift_mm(window,frame,quad),1e-9)
        self.assertAlmostEqual(marker_drift_mm(window,frame,quad+[1,0]),2.,places=6)
    def test_changed_camera_binding_is_rejected(self):
        from agent.locked_cabinet_marker import marker_drift_mm
        with self.assertRaises(ValueError):marker_drift_mm(dict(calibration_sha256='a'),dict(calibration_sha256='b'),np.zeros((4,2)))
