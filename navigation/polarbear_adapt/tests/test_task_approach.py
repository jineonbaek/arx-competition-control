import unittest
import numpy as np
from task_approach import TaskApproach,TaskStopReview

def point(which='point_1'):return dict(id='t',type='task',point_id=which,pose=[.2,0,.3])
class OrientedTask(unittest.TestCase):
 def test_position_reached_but_wrong_heading_rotates_before_completion(self):
  c=TaskApproach([0,0,0],[0,0,0],[point()])
  v=c.step([.2,0,0],[.2,0,0],10.,10.,.1,[0,0,0])
  self.assertFalse(v.terminal_near);self.assertEqual(v.command[:2],(0.,0.));self.assertGreater(v.command[2],0.)
 def test_task_pose_is_in_map_frame_even_with_nonzero_odom_transform(self):
  p=point();p['pose']=[4.,2.,1.]
  c=TaskApproach([3.5,2,.5],[0,0,0],[p])
  v=c.step([4.,2.,1.],[.4,.1,.5],10.,10.,.1,[0,0,0])
  self.assertTrue(v.terminal_near);self.assertEqual(v.command,(0.,0.,0.))
 def test_translation_is_not_finished_at_special_ten_cm_radius(self):
  c=TaskApproach([0,0,0],[0,0,0],[point()])
  v=c.step([.13,0,.3],[.13,0,.3],10.,10.,.1,[0,0,0])
  self.assertFalse(v.terminal_near);self.assertTrue(any(v.command[:2]))
 def test_actual_position_and_heading_are_both_required(self):
  c=TaskApproach([0,0,0],[0,0,0],[point()])
  self.assertTrue(c.step([.18,0,.28],[.18,0,.28],10.,10.,.1,[0,0,0]).terminal_near)
 def test_jump_and_stale_authority_still_stop_the_direct_controller(self):
  c=TaskApproach([0,0,0],[0,0,0],[point()]);c.step([0,0,0],[0,0,0],10,10,.1,[0,0,0])
  with self.assertRaisesRegex(ValueError,'jump'):c.step([.25,0,0],[0,0,0],10.1,10.1,.1,[0,0,0])
  with self.assertRaisesRegex(ValueError,'stale'):c.step([0,0,0],[0,0,0],10,11,.1,[0,0,0])
 def test_only_task_points_can_use_this_controller(self):
  with self.assertRaises(ValueError):TaskApproach([0,0,0],[0,0,0],[dict(type='via',pose=[.2,0,0])])
class StopReview(unittest.TestCase):
 def test_t1_third_stationary_check_preserves_seven_cm_rule(self):
  p=point();p['pose']=[0,0,0];r=TaskStopReview(p)
  self.assertEqual([r.check([.0533,0,.0646],True) for _ in range(3)],['correct','correct','t1_fallback'])
 def test_t2_exact_seven_cm_yaw_and_unstable_never_fall_back(self):
  for which,pose,stable in [('point_2',[.0533,0,.0646],True),('point_1',[.07,0,0],True),('point_1',[.06,0,.10],True),('point_1',[.05,0,0],False)]:
   with self.subTest(which=which,pose=pose,stable=stable):
    p=point(which);p['pose']=[0,0,0];r=TaskStopReview(p)
    self.assertEqual([r.check(pose,stable) for _ in range(3)],['correct','correct','park'])
 def test_stationary_strict_tolerance_completes_without_extra_loops(self):
  p=point();p['pose']=[0,0,0];self.assertEqual(TaskStopReview(p).check([.04,0,.08],True),'arrived')
