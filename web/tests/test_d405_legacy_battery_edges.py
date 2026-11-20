import unittest
import cv2,numpy as np
from d405_battery_edges import detect_face,PerspectiveTracker,projective_geometry

def scene(q):
 im=np.full((480,640,3),225,np.uint8);q=np.array(q,np.int32)
 cv2.fillConvexPoly(im,np.array([q[3],q[2],[q[2,0],370],[q[3,0],370]]),(30,30,30))
 cv2.fillConvexPoly(im,q,(88,88,88))
 for frac in (.33,.67):
  a=q[0]*(1-frac)+q[1]*frac;b=q[3]*(1-frac)+q[2]*frac;cv2.line(im,tuple(a.astype(int)),tuple(b.astype(int)),(40,40,40),1)
 cv2.fillConvexPoly(im,np.array([[100,479],[140,270],[220,240],[280,479]]),(75,170,145))
 cv2.fillConvexPoly(im,np.array([[640,479],[515,265],[450,240],[400,479]]),(75,170,145))
 return im

class DetectionTests(unittest.TestCase):
 def test_two_perspective_faces_without_fixed_corners(self):
  for q in ([[260,90],[390,100],[490,210],[180,195]],[[270,70],[440,76],[500,170],[210,165]]):
   r=detect_face(scene(q));self.assertIsNotNone(r)
   self.assertLess(np.max(np.linalg.norm(np.array(r['quad_px'])-q,axis=1)),6.)
 def test_empty_scene_no_fit(self):self.assertIsNone(detect_face(np.full((480,640,3),230,np.uint8)))
 def test_projective_center_is_not_bbox_center(self):
  q=np.array([[260,90],[390,100],[490,210],[180,195]],float);r=projective_geometry(q)
  H=np.array(r['H_unit_face_to_image']);unit=np.array([[[0,0],[1,0],[1,1],[0,1]]],np.float32)
  np.testing.assert_allclose(cv2.perspectiveTransform(unit,H)[0],q,atol=.001)
 def test_invalid_quad_rejected(self):
  with self.assertRaises(ValueError):projective_geometry([[0,0],[100,100],[100,0],[0,100]])

class TrackingTests(unittest.TestCase):
 def setUp(self):self.q=np.array([[260,90],[390,100],[490,210],[180,195]],float)
 def obs(self,q):return dict(quad_px=np.asarray(q).tolist(),edge_rms_px=[.5]*4)
 def test_spike_rejected_and_missing_expire(self):
  t=PerspectiveTracker()
  for i in range(5):r=t.update(self.obs(self.q),i*.1,i)
  r=t.update(self.obs(self.q+40),.5,5);self.assertEqual(r['tracking'],'held_recent')
  self.assertLess(np.max(abs(np.array(r['quad_px'])-self.q)),.1)
  self.assertIsNone(t.update(None,1.2,6))
 def test_same_frame_does_not_create_consensus(self):
  t=PerspectiveTracker();a=t.update(self.obs(self.q),1.,1);b=t.update(self.obs(self.q),1.,1)
  self.assertEqual(a['samples'],b['samples'])
 def test_real_motion_reacquired(self):
  t=PerspectiveTracker()
  for i in range(5):t.update(self.obs(self.q),i*.1,i)
  for i in range(5,8):r=t.update(self.obs(self.q+[25,0]),i*.1,i)
  self.assertLess(np.max(abs(np.array(r['quad_px'])-(self.q+[25,0]))),1.)
if __name__=='__main__':unittest.main()

