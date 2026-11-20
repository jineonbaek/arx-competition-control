"""T2 requirements use actual chassis heading, not map or target heading."""
import unittest,math,numpy as np
from task_approach import TaskApproach,TaskStopReview

def point(yaw=0.,which='point_2'):return dict(id='t2',type='task',point_id=which,pose=[0.,0.,yaw])
class T2Parking(unittest.TestCase):
 def test_first_strict_check_does_not_accept_three_cm_and_two_degrees(self):
  self.assertEqual(TaskStopReview(point()).check([.03,0.,math.radians(2)],True),'correct')
 def test_second_unsuccessful_precision_check_uses_existing_range(self):
  r=TaskStopReview(point());p=[.03,-.001,math.radians(2)]
  self.assertEqual([r.check(p,True) for _ in range(2)],['correct','t2_fallback'])
 def test_second_check_can_accept_two_cm_at_one_degree_without_legacy_fallback(self):
  r=TaskStopReview(point());p=[.015,-.001,math.radians(.5)]
  self.assertEqual([r.check(p,True) for _ in range(2)],['correct','arrived'])
 def test_left_never_passes_even_after_precision_budget(self):
  r=TaskStopReview(point())
  for _ in range(5):self.assertNotIn(r.check([0.,.004,0.],True),('arrived','t2_fallback'))
 def test_left_bias_inside_one_cm_still_commands_body_right(self):
  c=TaskApproach([0,.004,0],[0,.004,0],[point()]);r=c.step([0,.004,0],[0,.004,0],1.,1.2,.1,[0,0,0])
  self.assertFalse(r.terminal_near);self.assertLess(r.command[1],0.)
 def test_actual_heading_not_target_heading_defines_left(self):
  p=point(math.radians(4));r=TaskStopReview(p)
  for _ in range(3):self.assertNotIn(r.check([.04,.002,0.],True),('arrived','t2_fallback'))
  c=TaskApproach([.04,.002,0],[.04,.002,0],[p]);v=c.step([.04,.002,0],[.04,.002,0],1.,1.2,.1,[0,0,0]);self.assertLess(v.command[1],0.)
 def test_three_degree_yaw_is_not_precise_arrival(self):
  c=TaskApproach([0,0,0],[0,0,0],[point()]);v=c.step([0,0,math.radians(3)],[0,0,math.radians(3)],1.,1.2,.1,[0,0,0])
  self.assertFalse(v.terminal_near);self.assertLess(v.command[2],0.)
 def test_right_eight_mm_and_half_degree_can_finish_first_round(self):
  p=[0.,-.008,math.radians(.5)];self.assertEqual(TaskStopReview(point()).check(p,True),'arrived')
 def test_unstable_pose_never_hands_over_under_fallback(self):
  r=TaskStopReview(point())
  for _ in range(4):self.assertNotIn(r.check([0,-.004,0],False),('arrived','t2_fallback'))
 def test_nonstationary_timeouts_do_not_consume_precise_rounds(self):
  r=TaskStopReview(point());sample=[.03,-.001,.04]
  self.assertEqual(r.check(sample,False),'correct');self.assertEqual(r.check(sample,False),'correct')
  self.assertEqual(r.checks,0);self.assertFalse(r.arrival_policy.fallback)
  self.assertEqual(r.check(sample,True),'correct');self.assertEqual(r.checks,1)
 def test_t1_existing_four_cm_four_degree_tolerance_is_unchanged(self):
  self.assertEqual(TaskStopReview(point(which='point_1')).check([.04,0,.08],True),'arrived')
if __name__=='__main__':unittest.main()
