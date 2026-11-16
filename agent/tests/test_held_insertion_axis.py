import unittest,numpy as np
from scipy.spatial.transform import Rotation
from agent.held_insertion_path import plan_insert

class Model:
    reach=1.
    def wrist_transform(self,q,h):
        T=np.eye(4);T[:3,3]=q[:3];T[:3,:3]=Rotation.from_euler('xyz',q[3:]).as_matrix();return T
    def inverse_wrist(self,q,h,xyz,rpy,limits):return np.r_[np.array(xyz)/1000,np.radians(rpy)]
    def check(self,a,b,h):return dict(clear=True,lower_mm=100.)

class HeldAxisTests(unittest.TestCase):
    def test_horizontal_aperture_translation_preserves_observed_small_yaw(self):
        q=np.r_[0.,0.,.5,0.,0.,np.radians(2.)];R=Model().wrist_transform(q,10.)[:3,:3]
        path=plan_insert(Model(),q,10.,[0,0,500],[50,0,500],R,np.eye(4),[[-180,180]]*6,fixed_z_axis=[1.,0.,0.])
        self.assertLess(path['interpolation_position_bound_mm'],1.)
        self.assertEqual(np.ptp(np.array(path['tool_points_mm'])[:,2]),0.)
        np.testing.assert_allclose(path['joint_waypoints_rad'][0],q,atol=1e-8)
    def test_nonhorizontal_axis_cannot_claim_fixed_z(self):
        with self.assertRaises(ValueError):
            plan_insert(Model(),np.zeros(6),10.,[0,0,0],[0,0,50],np.eye(3),np.eye(4),[[-180,180]]*6,fixed_z_axis=[0.,0.,1.])
