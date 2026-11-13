import unittest
import numpy as np
try:
 from submap_core import compensate, persistent_points, spatial_split
except ImportError:
 compensate=persistent_points=spatial_split=None
class SubmapTests(unittest.TestCase):
 def test_time_compensation(self):
  self.assertIsNotNone(compensate)
  pose=np.eye(4);pose[:3,3]=[1,0,0]
  np.testing.assert_allclose(compensate(np.array([[2.,0,0]]),pose,np.eye(4)),[[3,0,0]])
 def test_reject_transient_and_near_body(self):
  self.assertIsNotNone(persistent_points)
  frames=[np.array([[2,0,0],[.2,0,0],[3+i,1,0]],float) for i in range(5)]
  out=persistent_points(frames,min_frames=3)
  np.testing.assert_allclose(out,[[2,0,0]])
 def test_spatial_blocks_disjoint(self):
  self.assertIsNotNone(spatial_split)
  p=np.array([[x*.1,y*.1,0] for x in range(40) for y in range(40)])
  tr,va=spatial_split(p)
  a=set(map(tuple,np.floor(tr/.6).astype(int)));b=set(map(tuple,np.floor(va/.6).astype(int)))
  self.assertTrue(a and b);self.assertFalse(a&b)
if __name__=='__main__':unittest.main()
