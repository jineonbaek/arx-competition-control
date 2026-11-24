import unittest
import numpy as np
from special_segments import StraightChain
class StartAssist(unittest.TestCase):
 def setUp(self):
  self.c=StraightChain([0,0,0],[0,0,0],[dict(id='p',type='special',pose=[.2,0,0])]);self.stamp=0.;self.previous=[0,0,0]
 def step(self,x=0):
  self.stamp+=.1;r=self.c.step([x,0,0],[x,0,0],self.stamp,self.stamp,.1,self.previous);self.previous=r.command;return np.linalg.norm(r.command[:2])
 def test_stalled_floor_steps_point_three_then_point_six_and_caps(self):
  self.step();self.assertAlmostEqual(self.step(),.225)
  self.assertEqual(self.c.request_start_assist(),.3)
  for _ in range(4):value=self.step()
  self.assertAlmostEqual(value,.3)
  self.assertEqual(self.c.request_start_assist(),.6)
  for _ in range(5):value=self.step()
  self.assertAlmostEqual(value,.6);self.assertEqual(self.c.request_start_assist(),.6)
 def test_measured_translation_exits_boost_and_restores_terminal_slowdown(self):
  self.step();self.c.request_start_assist();self.c.request_start_assist()
  for _ in range(5):self.step()
  for _ in range(6):value=self.step(.016)
  self.assertEqual(self.c.start_assist_floor,0.);self.assertAlmostEqual(value,.225)
 def test_terminal_still_returns_zero_and_new_recovery_can_reset_assist(self):
  self.step();self.c.request_start_assist();self.c.reset_start_assist();self.assertEqual(self.c.start_assist_floor,0.)
  self.c.request_start_assist();self.stamp+=.2
  r=self.c.step([.11,0,0],[.11,0,0],self.stamp,self.stamp,.2,self.previous)
  self.assertTrue(r.terminal_near);self.assertEqual(r.command,(0.,0.,0.));self.assertEqual(self.c.start_assist_floor,0.)
if __name__=='__main__':unittest.main()
