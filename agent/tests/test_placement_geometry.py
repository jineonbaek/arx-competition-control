import time
import unittest
import cv2
import numpy as np
from agent import placement_geometry as g

def frame(radius=180):
    n=460; yy,xx=np.mgrid[:n,:n]; xyz=np.stack([(xx-230)*.001,(yy-230)*.001,np.ones((n,n))],axis=-1)
    rgb=np.full((n,n,3),120,np.uint8);cv2.circle(rgb,(230,230),radius,(255,0,0),8)
    return dict(id='synthetic',camera_id='d435',image_time=time.time(),arm_time=time.time(),body_time=time.time(),
        xyz=xyz,rgb=rgb,base_from_color=np.diag([1.,-1.,-1.,1.])+np.array([[0,0,0,0],[0,0,0,0],[0,0,0,1],[0,0,0,0]]),calibration_sha256='a'*64)

class PlacementGeometryTests(unittest.TestCase):
    def test_metric_left_and_complete_footprint(self):
        self.assertTrue(hasattr(g,'analyze'),'red-ring RGB-D analyzer is required')
        r=g.analyze(frame()); self.assertEqual(r['status'],'localized');self.assertGreater(r['target_base_m'][1],r['center_base_m'][1])
        self.assertAlmostEqual(r['radius_m'],.18,delta=.01);self.assertLessEqual(r['xy_tolerance_mm'],30)
        for p in r['footprint_base_m']:
            self.assertLess(np.linalg.norm(np.array(p)-r['center_base_m']),r['radius_m'])
    def test_empty_red_disk_is_not_ring(self):
        f=frame();cv2.circle(f['rgb'],(230,230),180,(255,0,0),-1)
        with self.assertRaisesRegex(ValueError,'闭环|红圈'):g.analyze(f)
    def test_two_rings_are_ambiguous(self):
        f=frame(65);cv2.circle(f['rgb'],(65,65),45,(255,0,0),6)
        with self.assertRaisesRegex(ValueError,'唯一'):g.analyze(f)
    def test_no_or_zero_depth_rejected(self):
        f=frame();f['xyz'][:]=0
        with self.assertRaises(ValueError):g.analyze(f)
    def test_stale_future_and_unpaired_frames_rejected(self):
        for name,offset in [('image_time',-20),('image_time',20),('body_time',-1)]:
            f=frame();f[name]+=offset
            with self.assertRaises(ValueError):g.analyze(f)
    def test_nan_and_camera_mismatch_rejected(self):
        f=frame();f['xyz'][0,0]=np.nan
        with self.assertRaises(ValueError):g.analyze(f)
        f=frame();f['camera_id']='astra'
        with self.assertRaises(ValueError):g.analyze(f)
    def test_small_circle_cannot_fit_two_batteries(self):
        with self.assertRaisesRegex(ValueError,'空间|尺寸'):g.analyze(frame(60))
    def test_occupied_left_rejected(self):
        f=frame();f['xyz'][120:175,205:255,2]=.88
        with self.assertRaisesRegex(ValueError,'占用|支撑'):g.analyze(f)
    def test_raise_plane_slope_rejected(self):
        f=frame();f['xyz'][:,:,2]+=f['xyz'][:,:,0]*.5
        with self.assertRaisesRegex(ValueError,'平面|倾斜'):g.analyze(f)
    def test_depth_not_unit_scaled_to_mm(self):
        f=frame();f['xyz']*=1000
        with self.assertRaises(ValueError):g.analyze(f)
    def test_looser_horizontal_does_not_relax_vertical(self):
        r=g.analyze(frame());self.assertEqual(r['z_tolerance_mm'],3);self.assertEqual(r['yaw_tolerance_deg'],8)

if __name__=='__main__':unittest.main()

class SupportCoverageTests(unittest.TestCase):
    def test_tiny_patch_of_depth_is_not_complete_footprint_support(self):
        f=frame();f['xyz'][110:188,170:290]=0
        with self.assertRaisesRegex(ValueError,'支撑|深度'):g.analyze(f)
    def test_occupied_verification_must_be_explicit(self):
        f=frame();f['xyz'][120:175,205:255,2]=.88
        r=g.analyze(f,allow_occupied=True);self.assertEqual(r['side'],'left')

class RotationEnvelopeTests(unittest.TestCase):
    def test_obstacle_at_rotated_extreme_corner_is_rejected(self):
        f=frame();r=g.analyze(f);half=np.array(r['battery_size_m'][:2])/2;tol=r['xy_tolerance_mm']/1000
        # At the +yaw extreme, just outside nominal half-X + translation tolerance.
        a=np.radians(8);x=half[0]*np.cos(a)+half[1]*np.sin(a)+tol-.002
        y=half[1]*np.cos(a)-half[0]*np.sin(a)+tol-.002
        target=np.array(r['target_base_m']);u=int(round((target[0]+x)/.001+230));v=int(round(230-(target[1]+y)/.001))
        f['xyz'][v-2:v+3,u-2:u+3,2]=.95
        with self.assertRaisesRegex(ValueError,'占用|支撑'):g.analyze(f)
    def test_displayed_footprint_perimeter_has_full_rectangle_area(self):
        r=g.analyze(frame());pts=np.array(r['footprint_base_m'])-r['target_base_m']
        q=np.column_stack((pts@r['forward_axis_base'],pts@r['left_axis_base']))
        area=abs(np.sum(q[:,0]*np.roll(q[:,1],-1)-q[:,1]*np.roll(q[:,0],-1)))/2
        self.assertAlmostEqual(area,np.prod(r['battery_size_m'][:2]),places=7)
