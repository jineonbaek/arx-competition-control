"""Oriented map-frame task approach; shared continuity/effort guards, no maps."""
import math
import numpy as np
from special_segments import StraightChain,Step,pose,wrap
from mppi_trial.effort_budget import bound
from mppi_trial.session_policy import slew_effort
from mppi_trial.trial_policy import speed_factors
from mppi_trial.speed_profile import SLOW_SDK,cap_sdk

POSITION=.045
YAW=.09

class TaskApproach(StraightChain):
 def __init__(self,entry_map,entry_odom,points):
  if len(points)!=1 or points[0].get('type')!='task' or points[0].get('point_id') not in ('point_1','point_2'):raise ValueError('invalid direct task')
  super().__init__(entry_map,entry_odom,points)
  self.goal=pose(points[0]['pose']);self.radius=POSITION
 def terminal_ok(self,mp,od):
  return np.linalg.norm(pose(mp)[:2]-self.goal[:2])<=POSITION and abs(wrap(mp[2]-self.goal[2]))<=YAW
 def step(self,mp,od,stamp,now,dt,previous):
  mp,od=self.sample(mp,od,stamp,now);previous=pose(previous)
  if not math.isfinite(dt) or not 0<dt<=.3:raise ValueError('special_cycle_overrun')
  error=float(np.linalg.norm(self.goal[:2]-mp[:2]));yaw=wrap(self.goal[2]-mp[2])
  if self.terminal_ok(mp,od):return Step((0.,0.,0.),[],True)
  raw=np.zeros(3)
  if error>POSITION:
   line=self.goal[:2]-self.entry_map[:2];length=float(np.linalg.norm(line));unit=line/max(length,1e-12)
   along=float(np.clip((mp[:2]-self.entry_map[:2])@unit,0.,length))
   direction=self.entry_map[:2]+unit*min(length,along+.15)-mp[:2]
   if np.linalg.norm(direction)<1e-9:direction=self.goal[:2]-mp[:2]
   c,s=math.cos(mp[2]),math.sin(mp[2]);raw[:2]=np.array([[c,s],[-s,c]])@direction/max(float(np.linalg.norm(direction)),1e-12)*min(SLOW_SDK,max(min(.14,SLOW_SDK),1.4*(error-POSITION)))
  # Commissioned turn-start effort. Slew and combined-effort bounds still apply.
  raw[2]=0. if abs(yaw)<=YAW else math.copysign(.10,yaw)
  xy,w=speed_factors(self.measured);raw[:2]*=xy;raw[2]*=w
  command=cap_sdk(bound(slew_effort(previous,bound(raw),dt)),'slow')
  return Step(tuple(map(float,command)),[],False)

class TaskStopReview:
 def __init__(self,point):self.point=point;self.goal=pose(point['pose']);self.checks=0
 def check(self,mp,stationary):
  self.checks+=1;mp=pose(mp);xy=float(np.linalg.norm(mp[:2]-self.goal[:2]));yaw=abs(wrap(mp[2]-self.goal[2]))
  if stationary and xy<=POSITION and yaw<=YAW:return 'arrived'
  if self.checks>=3:
   if stationary and self.point.get('point_id')=='point_1' and xy<.07 and yaw<=YAW:return 't1_fallback'
   return 'park'
  return 'correct'
