import unittest,cv2,numpy as np
from rim_edges import detect_rim,line_angle_error,RimTracker
class Tests(unittest.TestCase):
 def scene(self,occlude=False):
  im=np.full((480,640,3),230,np.uint8)
  cv2.fillConvexPoly(im,np.array([[200,60],[460,75],[450,220],[180,205]]),(210,15,15))
  cv2.fillConvexPoly(im,np.array([[215,80],[443,93],[434,199],[200,187]]),(230,230,230))
  if occlude:im[140:240,160:490]=40
  return im
 def test_full_four_straight_edges(self):
  r=detect_rim(self.scene());self.assertEqual(set(r['edges']),{'top','bottom','left','right'});self.assertIsNotNone(r['quad_px'])
 def test_occlusion_not_fabricated_as_bottom(self):
  r=detect_rim(self.scene(True));self.assertIn('top',r['edges']);self.assertNotIn('bottom',r['edges']);self.assertIsNone(r['quad_px'])
 def test_no_red(self):self.assertIsNone(detect_rim(np.zeros((480,640,3),np.uint8)))
 def test_angle_parallel_and_wrap(self):
  self.assertAlmostEqual(line_angle_error([[0,0],[10,1]],[[20,20],[30,21]]),0.)
  self.assertAlmostEqual(line_angle_error([[0,0],[10,0]],[[0,0],[10,1]]),-5.710593,places=5)
  self.assertAlmostEqual(line_angle_error([[10,0],[0,0]],[[0,0],[10,0]]),0.)
 def test_tracker_expires(self):
  t=RimTracker();r=t.update(detect_rim(self.scene()),0.,1);self.assertIn('top',r['edges']);self.assertIsNone(t.update(None,1.,2))
if __name__=='__main__':unittest.main()
