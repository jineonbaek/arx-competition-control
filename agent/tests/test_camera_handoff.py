import json
import unittest
import numpy as np
from foundationpose.camera_geometry import rectify, cross_camera_fit
from agent.tests.test_grasp_integration import frame

class Pinhole:
    class distortion:
        none = 0
    class intrinsics: pass
    @staticmethod
    def rs2_project_point_to_pixel(k,p):
        return [k.fx*p[0]/p[2]+k.ppx,k.fy*p[1]/p[2]+k.ppy]

class CameraHandoffTests(unittest.TestCase):
    def head(self):
        f=frame();f['camera_id']=np.array('d435')
        k=json.loads(str(f['intrinsics']));k.update(model='none',coeffs=[0.]*5)
        f['intrinsics']=np.array(json.dumps(k))
        return f
    def test_rectification_preserves_pinhole_pixels_and_depth_units(self):
        f=self.head();k=json.loads(str(f['intrinsics']))
        rgb,z,m,K,maps,intr=rectify(f['rgb'],f['xyz'][...,2],np.ones((20,30),bool),k,Pinhole)
        np.testing.assert_array_equal(rgb,f['rgb'])
        np.testing.assert_allclose(z,.4);self.assertTrue(m.all())
        np.testing.assert_allclose(K,[[30,0,15],[0,30,10],[0,0,1]])
    def test_reprojection_uses_head_transform_not_wrist_transform(self):
        f=self.head();f['base_from_color'][:3,3]=[1.,2.,3.]
        pose=np.eye(4);pose[:3,3]=[1.,2.,3.4]
        def render(K,h,w,T):
            np.testing.assert_allclose(T[:3,3],[0,0,.4],atol=1e-8)
            return np.full((h,w),T[2,3])
        r=cross_camera_fit(f,np.ones((20,30)),pose,Pinhole,render)
        self.assertEqual(r['quality']['status'],'consistent')
        self.assertFalse(r['identity_confirmed']);self.assertFalse(r['executable'])
        self.assertEqual(r['head_snapshot_id'],'a'*32)
    def test_wrong_depth_or_other_battery_is_rejected(self):
        f=self.head();pose=np.eye(4);pose[2,3]=.6
        r=cross_camera_fit(f,np.ones((20,30)),pose,Pinhole,
            lambda K,h,w,T:np.full((h,w),T[2,3]))
        self.assertEqual(r['quality']['status'],'rejected')
        self.assertAlmostEqual(r['quality']['median_abs_depth_mm'],200,places=3)
    def test_head_camera_range_extends_to_three_metres(self):
        f=self.head();f['xyz'][...,2]=2.;f['raw_depth'][:]=2000
        r=cross_camera_fit(f,np.ones((20,30)),np.eye(4),Pinhole,
            lambda K,h,w,T:np.full((h,w),2.))
        self.assertEqual(r['quality']['status'],'consistent')
    def test_wrong_camera_bad_mask_or_transform_rejected(self):
        f=self.head();f['camera_id']=np.array('d405')
        render=lambda K,h,w,T:np.ones((h,w))
        with self.assertRaises(ValueError):cross_camera_fit(f,np.ones((20,30)),np.eye(4),Pinhole,render)
        f=self.head()
        for mask,pose in ((np.ones((2,3)),np.eye(4)),(np.ones((20,30)),np.zeros((4,4)))):
            with self.assertRaises(ValueError):cross_camera_fit(f,mask,pose,Pinhole,render)

if __name__=='__main__':unittest.main()
