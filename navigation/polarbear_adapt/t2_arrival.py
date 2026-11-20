"""T2-only stopping policy. Distances are metres; heading defines body-left."""
import math
import numpy as np
from special_segments import pose,wrap

PRECISE=.01
ACCEPTABLE=.02
PRECISE_YAW=math.radians(1.)
LEGACY_POSITION=.045
LEGACY_YAW=.09
RIGHT_BIAS=.005

def left_error(current,goal):
 current,goal=pose(current),pose(goal);d=current[:2]-goal[:2]
 return float(-math.sin(current[2])*d[0]+math.cos(current[2])*d[1])

class T2Arrival:
 def __init__(self,goal):
  self.goal=pose(goal);self.checks=0;self.fallback=False;self.approved=None
 def within(self,current,position,yaw):
  current=pose(current)
  return (np.linalg.norm(current[:2]-self.goal[:2])<=position
          and abs(wrap(current[2]-self.goal[2]))<=yaw and left_error(current,self.goal)<=0.)
 def terminal_ok(self,current):
  return self.within(current,LEGACY_POSITION if self.fallback else PRECISE,LEGACY_YAW if self.fallback else PRECISE_YAW)
 def check(self,current,stationary):
  if not stationary:return 'correct'
  self.checks+=1
  if not self.fallback and stationary and self.within(current,PRECISE,PRECISE_YAW):
   self.approved=(PRECISE,PRECISE_YAW);return 'arrived'
  if not self.fallback and self.checks>=2:
   if stationary and self.within(current,ACCEPTABLE,PRECISE_YAW):
    self.approved=(ACCEPTABLE,PRECISE_YAW);return 'arrived'
   self.fallback=True
  if self.fallback and stationary and self.within(current,LEGACY_POSITION,LEGACY_YAW):
   self.approved=(LEGACY_POSITION,LEGACY_YAW);return 't2_fallback'
  return 'park' if self.checks>=3 else 'correct'
 def handoff_ok(self,current):
  limits=self.approved or (PRECISE,PRECISE_YAW)
  return self.within(current,*limits)
 def revoke(self):
  self.approved=None
  if self.checks>=2:self.fallback=True
