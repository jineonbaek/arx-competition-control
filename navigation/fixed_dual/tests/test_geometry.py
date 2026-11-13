import sys, unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry import transforms, compensate
class Geometry(unittest.TestCase):
 def test_axes_origin_and_relative(self):
  front,rear=transforms()
  np.testing.assert_allclose(rear[:3,3],[-.2832,0,.228])
  np.testing.assert_allclose(rear[:3,:3],[[0,-1,0],[1,0,0],[0,0,1]],atol=1e-15)
  rel=np.linalg.inv(front)@rear
  np.testing.assert_allclose(rel[:3,3],[-.46853,.000003,-.002],atol=1e-15)
  p=np.array([.2,.3,.4,1.]);np.testing.assert_allclose(front@rel@p,rear@p)
 def test_time_compensation_preserves_stationary_world(self):
  from scipy.spatial.transform import Rotation
  sensor=transforms()[1];world=np.array([[2.,1.,.6],[3.,-.2,.3]])
  poses=np.tile(np.eye(4),(2,1,1));poses[1,:3,:3]=Rotation.from_euler('z',.3).as_matrix();poses[1,:3,3]=[.1,.04,0]
  raw=np.array([(p-T[:3,3])@T[:3,:3] for p,T in zip(world,poses@sensor)])
  out=compensate(raw,poses,poses[-1],sensor)
  last=poses[-1]@sensor;np.testing.assert_allclose(out@last[:3,:3].T+last[:3,3],world,atol=1e-12)
if __name__=='__main__':unittest.main()
