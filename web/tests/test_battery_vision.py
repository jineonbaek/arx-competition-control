import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))

class BatteryVisionTests(unittest.TestCase):
 def module(self):
  self.assertIsNotNone(importlib.util.find_spec('battery_vision'),'Battery detector not implemented')
  import battery_vision
  return battery_vision
 def scene(self):
  im=np.full((480,640,3),35,np.uint8)
  for x in [180,430]:
   cv2.rectangle(im,(x-55,90),(x+55,320),(255,255,255),-1)
   cv2.rectangle(im,(x-20,280),(x+20,294),(100,100,100),-1)
  return im
 def test_leftmost_slot_is_found_without_fixed_pixel_coordinates(self):
  m=self.module();im=self.scene();r=m.detect_handle(im)
  self.assertIsNotNone(r);self.assertLess(abs(r['hole_px'][0]-180),3)
  self.assertGreater(r['bar_px'][1],r['hole_px'][1])
  self.assertIsNone(m.detect_handle(np.full_like(im,35)))
 def test_left_dimmer_body_is_not_skipped_for_bright_right_battery(self):
  m=self.module();im=self.scene()
  cv2.rectangle(im,(125,90),(235,320),(230,230,230),-1)
  cv2.rectangle(im,(125,270),(235,320),(255,255,255),-1)
  cv2.rectangle(im,(160,280),(200,294),(100,100,100),-1)
  r=m.detect_handle(im)
  self.assertIsNotNone(r)
  self.assertLess(r['hole_px'][0],250,'Must not substitute the right battery for the dimmer left battery')
 def test_depth_uses_solid_bar_not_hole_background_and_fails_closed(self):
  m=self.module();im=self.scene();r=m.detect_handle(im)
  depth=np.full((480,640),.7,np.float32);depth[279:296,159:202]=2.
  intr={'fx':600.,'fy':600.,'cx':320.,'cy':240.}
  p=m.locate_handle(r,im,depth,intr)
  self.assertEqual(p['status'],'surface_valid');self.assertAlmostEqual(p['bar_surface_xyz_m'][2],.7,places=3)
  self.assertIsNone(p['robot_xyz_m'])
  self.assertNotEqual(p['method'],'hole_center_depth')
  missing=m.locate_handle(r,im,np.zeros_like(depth),intr)
  self.assertEqual(missing['status'],'depth_invalid');self.assertIsNone(missing['bar_surface_xyz_m'])
 def test_stale_or_unsynchronized_frames_never_return_valid_depth(self):
  m=self.module()
  self.assertFalse(m.frames_usable(1000,1100,1.0))
  self.assertFalse(m.frames_usable(1000,1001,1.0,age_s=1.0))
  self.assertTrue(m.frames_usable(1000,1001,1.0,age_s=.01))

if __name__=='__main__':unittest.main()
