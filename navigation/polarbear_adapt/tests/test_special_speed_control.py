"""Controller regressions: braking, latency and lateral damping; no hardware."""
import unittest
import numpy as np
from special_segments import StraightChain

class SpeedControl(unittest.TestCase):
 def chain(self,points=None):return StraightChain([0.,0.,0.],[0.,0.,0.],points or [dict(id='goal',pose=[1.,0.,0.])])
 def test_near_terminal_reduces_effort_before_arrival_radius(self):
  c=self.chain();r=c.step([.75,0,0],[.75,0,0],1.,1.25,.05,[.6,0,0])
  self.assertLess(r.command[0],.6);self.assertGreater(r.command[0],0.);self.assertFalse(r.terminal_near)
 def test_long_straight_retains_normal_cruise(self):
  c=self.chain([dict(id='goal',pose=[4.,0.,0.])]);r=c.step([1.,0,0],[1.,0,0],1.,1.25,.05,[.6,0,0]);self.assertAlmostEqual(r.command[0],.6)
 def test_corner_slows_but_collinear_midpoint_retains_cruise(self):
  corner=self.chain([dict(id='a',pose=[1.,0,0]),dict(id='b',pose=[1.,1.,0])]);straight=self.chain([dict(id='a',pose=[1.,0,0]),dict(id='b',pose=[2.,0,0])])
  args=([.85,0,0],[.85,0,0],1.,1.25,.05,[.6,0,0])
  self.assertLess(corner.step(*args).command[0],straight.step(*args).command[0])
 def test_lateral_momentum_is_damped_instead_of_using_pose_error_alone(self):
  a=self.chain([dict(id='goal',pose=[4.,0,0])]);b=self.chain([dict(id='goal',pose=[4.,0,0])])
  a.previous_sample=(.9,np.array([.96,.01,0]),np.array([.96,.01,0]));b.previous_sample=(.9,np.array([.96,.02,0]),np.array([.96,.02,0]))
  ra=a.step([1.,.02,0],[1.,.02,0],1.,1.25,.05,[.6,0,0]);rb=b.step([1.,.02,0],[1.,.02,0],1.,1.25,.05,[.6,0,0])
  self.assertLess(ra.command[1],rb.command[1])
 def test_larger_source_age_does_not_request_more_approach_effort(self):
  outputs=[]
  for age in (.05,.45):
   c=self.chain();c.previous_sample=(.9,np.array([.46,0,0]),np.array([.46,0,0]));r=c.step([.5,0,0],[.5,0,0],1.,1.+age,.05,[.3,0,0]);outputs.append(r.command[0])
  self.assertLess(outputs[1],outputs[0])
 def test_terminal_recovery_cannot_return_to_normal_cruise(self):
  c=self.chain();c.terminal_correction=True;r=c.step([1.2,0,0],[1.2,0,0],1.,1.2,.05,[0,0,0]);self.assertLessEqual(np.linalg.norm(r.command[:2]),.14+1e-9)
  r=c.step([1.2,0,0],[1.2,0,0],1.1,1.3,.05,r.command);self.assertLessEqual(np.linalg.norm(r.command[:2]),.14+1e-9)
if __name__=='__main__':unittest.main()
