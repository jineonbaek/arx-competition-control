import unittest
import numpy as np
from scipy.spatial.transform import Rotation
try:
 from relative import interpolate, relative_pose, world_alignment
except ImportError:
 interpolate=relative_pose=world_alignment=None

class RelativeTests(unittest.TestCase):
 def test_implementation_exists(self):
  self.assertTrue(callable(interpolate),'same-time relative pose core not implemented')
 def test_slerp_and_no_extrapolation(self):
  if interpolate is None:self.skipTest('core not implemented')
  ts=np.array([1700000000000000000,1700000000100000000],dtype=np.int64)
  ps=np.array([[0,0,0],[2,0,0]],float)
  qs=Rotation.from_euler('z',[179,-179],degrees=True).as_quat()
  m=interpolate(ts,ps,qs,int(ts[0])+50000000)
  np.testing.assert_allclose(m[:3,3],[1,0,0]);self.assertAlmostEqual(abs(Rotation.from_matrix(m[:3,:3]).as_euler('xyz',degrees=True)[2]),180)
  with self.assertRaises(ValueError):interpolate(ts,ps,qs,int(ts[0])-1)
  with self.assertRaises(ValueError):interpolate(ts[::-1],ps,qs,int(ts[0]))
  with self.assertRaises(ValueError):interpolate(ts,ps,qs,int(ts[0])+50000000,max_gap=.05)
 def test_known_dynamic_motion_and_gauge(self):
  if relative_pose is None:self.skipTest('core not implemented')
  b0=np.eye(4);h0=np.eye(4);h0[:3,3]=[0,0,1]
  wb=np.eye(4);wb[:3,3]=[5,2,0];wb[:3,:3]=Rotation.from_euler('z',.4).as_matrix()
  wh=np.eye(4);wh[:3,3]=[-2,4,1];wh[:3,:3]=Rotation.from_euler('z',-.6).as_matrix()
  align=world_alignment(wb@b0,wh@h0,h0)
  rel=h0.copy();rel[:3,:3]=Rotation.from_euler('x',.08).as_matrix();rel[0,3]=.02
  motion=np.eye(4);motion[:3,3]=[1,0,0];motion[:3,:3]=Rotation.from_euler('z',.2).as_matrix()
  result=relative_pose(wb@motion,wh@motion@rel,align)
  np.testing.assert_allclose(result,rel,atol=1e-12)

if __name__=='__main__':unittest.main()
