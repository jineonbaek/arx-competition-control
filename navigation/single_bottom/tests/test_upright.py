import unittest
import numpy as np
try:
 from core import body_state, initial_gauge, updated_lower_mesh_transform, upright_config
except ImportError:
 body_state=initial_gauge=updated_lower_mesh_transform=upright_config=None

class UprightTests(unittest.TestCase):
 def test_localization_seed_has_nonzero_uncertainty(self):
  import core
  self.assertTrue(hasattr(core,'localization_seed_covariance'))
  C=np.array(core.localization_seed_covariance()).reshape(6,6)
  self.assertGreater(C[0,0],0);self.assertGreater(C[1,1],0);self.assertGreater(C[5,5],0)
  self.assertTrue(np.isfinite(C).all())
 def test_saved_map_pose_must_be_explicit_and_finite(self):
  import core
  self.assertTrue(hasattr(core,'localization_initial_pose'))
  with self.assertRaises(ValueError):core.localization_initial_pose({})
  with self.assertRaises(ValueError):core.localization_initial_pose({'ARX_INITIAL_X':'nan','ARX_INITIAL_Y':'0','ARX_INITIAL_YAW':'0'})
  self.assertEqual(core.localization_initial_pose({'ARX_INITIAL_X':'1','ARX_INITIAL_Y':'-2','ARX_INITIAL_YAW':'.3'}),[1.,-2.,.3])
 def test_new_origins_and_sensor_internal_offset(self):
  self.assertIsNotNone(upright_config)
  c=upright_config();np.testing.assert_allclose(c['imu_origin_m'],[.196330,.023287,.185880]);np.testing.assert_allclose(np.array(c['lidar_origin_m'])-c['imu_origin_m'],[-.011,-.02329,.04412])
 def test_mesh_mount_face_flips_and_moves(self):
  self.assertIsNotNone(updated_lower_mesh_transform)
  old=np.eye(4);old[:3,:3]=np.diag([1,-1,-1]);old[:3,3]=[.197330,-.000003,.234]
  new=updated_lower_mesh_transform(old,old);np.testing.assert_allclose(new[:3,:3],np.eye(3));np.testing.assert_allclose(new[:3,3],[.185330,-.000003,.230])
  mount=np.array([0,0,-.047,1]);np.testing.assert_allclose((new@mount)[:3],[.185330,-.000003,.183])
 def test_lever_arm_pure_rotation_body_stationary(self):
  self.assertIsNotNone(body_state)
  c=upright_config();p=np.array(c['imu_origin_m']);w=np.array([0.,0.,1.]);v=np.cross(w,p)
  out=body_state(p,[0,0,0,1],np.r_[v,w],p);np.testing.assert_allclose(out[0],np.zeros(3),atol=1e-12);np.testing.assert_allclose(out[2][:3],0,atol=1e-12)
 def test_initial_gauge_sets_start_base_at_origin(self):
  self.assertIsNotNone(initial_gauge)
  T=np.eye(4);T[:3,3]=[2,3,4];T[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
  G=initial_gauge(T);np.testing.assert_allclose(G@T,np.eye(4),atol=1e-12)

if __name__=='__main__':unittest.main()
