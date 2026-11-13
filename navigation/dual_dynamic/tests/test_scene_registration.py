import unittest
import numpy as np
from scipy.spatial.transform import Rotation
try:
 from scene_registration import register,prepare_target
except ImportError:
 register=prepare_target=None

class RegistrationTests(unittest.TestCase):
 def test_known_transform_three_planes(self):
  self.assertIsNotNone(register,'registration not implemented')
  rng=np.random.default_rng(7);uv=rng.uniform(-2,2,(1200,2));cloud=np.vstack([np.c_[uv,np.zeros(1200)],np.c_[uv[:,0],np.zeros(1200),uv[:,1]],np.c_[np.zeros(1200),uv]])
  T=np.eye(4);T[:3,:3]=Rotation.from_rotvec([.02,-.015,.025]).as_matrix();T[:3,3]=[.04,-.03,.025]
  source=(cloud-T[:3,3])@T[:3,:3];out=register(source,prepare_target(cloud),np.eye(4))
  self.assertLess(np.linalg.norm(out['T'][:3,3]-T[:3,3]),.005)
  self.assertLess(Rotation.from_matrix(out['T'][:3,:3]@T[:3,:3].T).magnitude(),.003)
  self.assertEqual(out['rank'],6)
 def test_floor_degeneracy(self):
  self.assertIsNotNone(register,'registration not implemented')
  rng=np.random.default_rng(4);cloud=np.c_[rng.uniform(-2,2,(2000,2)),np.zeros(2000)]
  result=register(cloud,prepare_target(cloud),np.eye(4))
  self.assertLess(result['rank'],6)
if __name__=='__main__':unittest.main()
