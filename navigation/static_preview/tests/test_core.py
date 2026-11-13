import unittest, math
from core import level_points, obstacle_scan, valid_goal, split_ground
class CoreTests(unittest.TestCase):
 def test_gravity_rotation_keeps_sensor_origin(self):
  p=level_points([[1,0,0]], [0,0,math.sqrt(.5),math.sqrt(.5)])
  self.assertAlmostEqual(p[0][0],0,places=6);self.assertAlmostEqual(p[0][1],1,places=6)
 def test_scan_absence_not_free(self):
  r=obstacle_scan([[1,0,.1],[float('nan'),0,0],[2,0,-.23]])
  self.assertEqual(sum(math.isfinite(x) for x in r),1)
  self.assertTrue(all(math.isnan(x) or math.isfinite(x) for x in r))
 def test_ground_labels_cannot_remove_high_obstacles(self):
  ground,obstacles=split_ground([[1,0,-.23],[1,0,-.1],[1,0,float('nan')]])
  self.assertEqual(len(ground),1);self.assertEqual(len(obstacles),1)
 def test_stale_or_invalid_goal_rejected(self):
  self.assertFalse(valid_goal([1,2,0], 2., .1, 'preview_map'))
  self.assertFalse(valid_goal([float('nan'),2,0], .1,.1,'preview_map'))
  self.assertFalse(valid_goal([1,2,0],.1,.1,'map'))
  self.assertTrue(valid_goal([1,2,0],.1,.1,'preview_map'))
if __name__=='__main__': unittest.main()
