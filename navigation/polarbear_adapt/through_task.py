"""Cruise through ordinary specials, then use the unchanged task parking policy."""
import numpy as np
from special_segments import StraightChain,Step
from task_approach import TaskApproach

class ThroughTaskChain(TaskApproach):
 def __init__(self,mp,od,points,arrival_policy=None):
  super().__init__(mp,od,[points[-1]],arrival_policy=arrival_policy)
  self.transit=StraightChain(mp,od,points,continuous=True,terminal_precision=False)
  self.approaching=False
 def step(self,mp,od,stamp,now,dt,previous):
  if self.approaching:return super().step(mp,od,stamp,now,dt,previous)
  self.transit.previous_sample=self.previous_sample
  result=self.transit.step(mp,od,stamp,now,dt,previous)
  self.previous_sample=self.transit.previous_sample;self.measured=self.transit.measured
  if self.transit.index<len(self.transit.goals)-1:return result
  # Start task precision from the actual passing pose, without a stop boundary.
  self.approaching=True;self.entry_map=np.asarray(mp,dtype=float).copy()
  self.previous_sample=None
  approach=super().step(mp,od,stamp,now,dt,result.command)
  return Step(approach.command,result.passed_ids,approach.terminal_near)
