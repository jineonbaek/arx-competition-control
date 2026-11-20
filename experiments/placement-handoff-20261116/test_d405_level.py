import unittest,numpy as np
from scipy.spatial.transform import Rotation
from d405_level import level_from_d405,level_from_axis
class Model:
    def wrist_transform(self,q,h):
        T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(np.asarray(q)[3:]).as_matrix();return T
class LevelTests(unittest.TestCase):
    def data(self,angle=0.):
        R=np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])
        T=np.eye(4);T[:3,:3]=Rotation.from_rotvec([angle*np.pi/180,0.,0.]).as_matrix()@R
        return dict(camera="d405",image_time=10.,arm_time=10.,body_time=10.,joints=[0.]*6,lift_rad=3.,T_base_from_object=T.tolist(),quality="verified_pose",calibration_verified=True)
    def run_level(self,d):return level_from_d405(d,Model(),[0.]*6,3.,[[-180.,180.]]*6,now=10.)
    def test_aligned_pose_retains(self):
        r=self.run_level(self.data());self.assertEqual(r["action"],"hold");self.assertFalse(r["executable"])
    def test_tilted_pose_proposes_bounded_improvement(self):
        r=self.run_level(self.data(5.));self.assertEqual(r["action"],"correct");self.assertLess(r["predicted_tilt_deg"],5.);self.assertLessEqual(max(abs(v) for v in r["joint_delta_deg"]),3.0001);self.assertFalse(r["executable"])
    def test_axis_level_proposal_is_bounded_and_nonexecuting(self):
        theta=np.radians(5.);obs={"status":"repeatable_axis_candidate","axis_base":[np.cos(theta),0.,np.sin(theta)]}
        r=level_from_axis(obs,Model(),[0.]*6,3.,[[-180.,180.]]*6)
        self.assertEqual(r["action"],"correct");self.assertLess(r["predicted_tilt_deg"],5.);self.assertLessEqual(max(abs(x) for x in r["joint_delta_deg"]),3.0001);self.assertFalse(r["executable"])
    def test_axis_level_requires_a_valid_axis(self):
        with self.assertRaises(ValueError):level_from_axis({"status":"repeatable_axis_candidate","axis_base":[0,0,0]},Model(),[0.]*6,3.,[[-180.,180.]]*6)
    def test_unverified_or_stale_data_not_a_motion_target(self):
        for x in (self.data()|{"calibration_verified":False},self.data()|{"camera":"d435"},self.data()|{"image_time":0.},self.data()|{"joints":[.2,0,0,0,0,0]}):
            with self.assertRaises(ValueError):self.run_level(x)
if __name__=="__main__":unittest.main()
