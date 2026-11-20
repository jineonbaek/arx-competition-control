import unittest
from trial_policy import speed_factors,verified_arrival,target_pose,trial_envelope,trial_progress
class PolicyTests(unittest.TestCase):
 def test_relative_targets_follow_vehicle_axes_and_heading(self):
  import math
  p=target_pose((1.,2.,math.pi/2),(0.,.2,.3))
  self.assertAlmostEqual(p[0],.8);self.assertAlmostEqual(p[1],2.)
  self.assertAlmostEqual(p[2],math.pi/2+.3)
 def test_bounded_envelope_covers_lateral_and_rotation_trials(self):
  self.assertTrue(trial_envelope((.001,.19,.01),(0.,.22,0.)))
  self.assertFalse(trial_envelope((.08,.19,0.),(0.,.22,0.)))
  self.assertTrue(trial_envelope((.002,.001,-.22),(0.,0.,-.25)))
  self.assertFalse(trial_envelope((.06,0.,-.22),(0.,0.,-.25)))
  self.assertFalse(trial_envelope((0.,0.,.4),(0.,0.,-.25)))
 def test_progress_is_signed_in_the_requested_direction(self):
  self.assertTrue(trial_progress((0.,-.02,0.),(0.,-.22,0.)))
  self.assertFalse(trial_progress((0.,.02,0.),(0.,-.22,0.)))
  self.assertTrue(trial_progress((0.,0.,.03),(0.,0.,.25)))
 def test_overspeed_reduces_effort_without_discarding_goal(self):
  xy,yaw=speed_factors((.22*.8/.3*2,0.,1.6));self.assertAlmostEqual(xy,.5);self.assertAlmostEqual(yaw,.5)
  self.assertGreater(xy,0);self.assertGreater(yaw,0)
 def test_no_slowdown_below_limit(self):self.assertEqual(speed_factors((.02,.03,.01)),(1.,1.))
 def test_cancelled_or_near_distance_is_not_arrival(self):
  self.assertFalse(verified_arrival([5,0],[.01,0,0],[0,0,0],True))
  self.assertFalse(verified_arrival(None,[.01,0,0],[0,0,0],True))
 def test_arrival_requires_result_position_and_stability(self):
  self.assertTrue(verified_arrival([4,0],[.02,0,.01],[.001,0,.001],True))
  for error,drift,idle in [([.05,0,0],[0,0,0],True),([0,0,.1],[0,0,0],True),([0,0,0],[.01,0,0],True),([0,0,0],[0,0,0],False)]:
   self.assertFalse(verified_arrival([4,0],error,drift,idle))
if __name__=='__main__':unittest.main()
