import unittest
import numpy as np
from scipy.spatial.transform import Rotation
try:
 from flex_core import cad_pose, body_mask, information_projector, solve
except ImportError:
 cad_pose=body_mask=information_projector=solve=None
class FlexTests(unittest.TestCase):
 def test_cad_inverse_and_filter(self):
  self.assertIsNotNone(cad_pose)
  B=cad_pose([.208330,-.023293,.278120]);H=cad_pose([.017536,-.022293,1.236910]);T=np.linalg.inv(H)@B
  np.testing.assert_allclose(T[:3,3],[.190794,.001,.958790],atol=1e-9)
  pts=np.array([[0,0,.3],[.6,0,.3],[0,.6,.3],[0,0,1.5]])
  np.testing.assert_array_equal(body_mask(pts,np.eye(4),padding=.02),[True,False,False,False])
 def test_degenerate_directions(self):
  self.assertIsNotNone(information_projector)
  W,rank=information_projector(np.diag([10,10,0,0,0,10]))
  self.assertEqual(rank,3);np.testing.assert_allclose(W@np.array([0,0,1,1,1,0]),0)
 def test_no_scene_returns_exact_bottom(self):
  self.assertIsNotNone(solve)
  n=12;t=np.arange(n)*.2;B=np.tile(np.eye(4),(n,1,1));B[:,0,3]=t*.1;A=cad_pose([.017536,-.022293,1.23691]);H=B@A
  result=solve(t,B,H,[],cad_pose([.208330,-.023293,.27812]),A)
  np.testing.assert_allclose(result['output'],B);self.assertTrue(np.all(result['fallback']))
 def test_known_flex_scene_constraint(self):
  self.assertIsNotNone(solve)
  n=20;t=np.arange(n)*.2;B=np.tile(np.eye(4),(n,1,1));B[:,0,3]=t*.05;A=cad_pose([.017536,-.022293,1.23691]);AB=cad_pose([.208330,-.023293,.27812]);D=np.tile(A,(n,1,1));D[:,0,3]+=.01*np.sin(t*2);D[:,:3,:3]=Rotation.from_rotvec(np.c_[np.zeros(n),.01*np.sin(t*2),np.zeros(n)]).as_matrix()@A[:3,:3];H=B@D
  scenes=[{'index':i,'T':np.linalg.inv(D[i])@AB,'information':np.eye(6),'sigma':.005} for i in range(n)]
  r=solve(t,B,H,scenes,AB,A,max_nfev=30)
  self.assertLess(np.sqrt(np.mean((r['relative'][:,0,3]-D[:,0,3])**2)),.008)
  self.assertTrue(np.isfinite(r['output']).all())
 def test_gross_scene_outlier_rejected(self):
  n=12;t=np.arange(n)*.2;B=np.tile(np.eye(4),(n,1,1));A=cad_pose([.017536,-.022293,1.23691]);AB=cad_pose([.208330,-.023293,.27812]);H=B@A;bad=np.linalg.inv(A)@AB;bad[0,3]+=2.
  r=solve(t,B,H,[{'index':5,'T':bad,'information':np.eye(6),'sigma':.01}],AB,A)
  self.assertEqual(r['accepted_scenes'],0);np.testing.assert_allclose(r['output'],B)
 def test_rank_three_constraint_cannot_enable_fusion(self):
  n=12;t=np.arange(n)*.2;B=np.tile(np.eye(4),(n,1,1));A=cad_pose([.017536,-.022293,1.23691]);AB=cad_pose([.208330,-.023293,.27812]);H=B@A
  r=solve(t,B,H,[{'index':5,'T':np.linalg.inv(A)@AB,'information':np.diag([1,1,0,0,0,1]),'sigma':.01}],AB,A)
  self.assertTrue(np.all(r['fallback']));np.testing.assert_allclose(r['output'],B)
 def test_conflicting_factors_cannot_authorize_fusion(self):
  n=12;t=np.arange(n)*.2;B=np.tile(np.eye(4),(n,1,1));A=cad_pose([.017536,-.022293,1.23691]);AB=cad_pose([.208330,-.023293,.27812]);H=B@A;nom=np.linalg.inv(A)@AB;left=nom.copy();right=nom.copy();left[0,3]+=.1;right[0,3]-=.1
  scenes=[{'index':5,'T':T,'information':np.eye(6),'sigma':.001} for T in [left,right]]
  r=solve(t,B,H,scenes,AB,A)
  self.assertTrue(np.all(r['fallback']))
 def test_scene_time_is_not_snapped(self):
  n=12;t=np.arange(n)*.2;B=np.tile(np.eye(4),(n,1,1));A=cad_pose([.017536,-.022293,1.23691]);AB=cad_pose([.208330,-.023293,.27812]);H=B@A
  scenes=[{'index':5,'time':1.09,'T':np.linalg.inv(A)@AB,'information':np.eye(6),'sigma':.01}]
  r=solve(t,B,H,scenes,AB,A)
  self.assertAlmostEqual(r['scene_postfit'][0]['evaluation_time'],1.09)
if __name__=='__main__':unittest.main()
