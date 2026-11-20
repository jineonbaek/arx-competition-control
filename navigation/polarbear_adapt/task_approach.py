"""Oriented map-frame task approach; shared continuity/effort guards, no maps."""
import math
import numpy as np
from special_segments import StraightChain,Step,pose,wrap
from mppi_trial.effort_budget import bound
from mppi_trial.session_policy import slew_effort
from mppi_trial.trial_policy import speed_factors
from mppi_trial.speed_profile import SLOW_SDK,cap_sdk

from t2_arrival import T2Arrival,PRECISE_YAW,RIGHT_BIAS,left_error

POSITION=.045
YAW=.09

class TaskApproach(StraightChain):
 def __init__(self,entry_map,entry_odom,points,arrival_policy=None):
  if len(points)!=1 or points[0].get('type')!='task' or points[0].get('point_id') not in ('point_1','point_2'):raise ValueError('invalid direct task')
  super().__init__(entry_map,entry_odom,points)
  self.goal=pose(points[0]['pose']);self.radius=POSITION
  self.arrival_policy=(arrival_policy or T2Arrival(self.goal)) if points[0]['point_id']=='point_2' else None
 def terminal_ok(self,mp,od):
  if self.arrival_policy is not None:return self.arrival_policy.terminal_ok(mp)
  return np.linalg.norm(pose(mp)[:2]-self.goal[:2])<=POSITION and abs(wrap(mp[2]-self.goal[2]))<=YAW
 def step(self,mp,od,stamp,now,dt,previous):
  if self.arrival_policy is not None:return self._t2_step(mp,od,stamp,now,dt,previous)
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

 def _t2_step(self,mp,od,stamp,now,dt,previous):
  source_dt=dt if self.previous_sample is None else stamp-self.previous_sample[0]
  mp,od=self.sample(mp,od,stamp,now);previous=pose(previous)
  if not math.isfinite(dt) or not 0<dt<=.3:raise ValueError('special_cycle_overrun')
  if self.terminal_ok(mp,od):return Step((0.,0.,0.),[],True)
  c,s=math.cos(mp[2]),math.sin(mp[2]);left=left_error(mp,self.goal)
  error=float(np.linalg.norm(self.goal[:2]-mp[:2]));yaw=wrap(self.goal[2]-mp[2]);raw=np.zeros(3)
  limit=POSITION if self.arrival_policy.fallback else .01
  if error>limit or left>0.:
   # Aim just right of the saved point in the CURRENT body frame; no point edit.
   target=self.goal[:2]+RIGHT_BIAS*np.array([s,-c]);direction=np.array([[c,s],[-s,c]])@(target-mp[:2])
   magnitude=float(np.linalg.norm(direction));floor=.06 if error<.06 else .14
   effort=min(SLOW_SDK,max(min(floor,SLOW_SDK),1.4*magnitude))
   raw[:2]=direction/max(magnitude,1e-12)*effort-.25*self.measured[:2]
  yaw_limit=YAW if self.arrival_policy.fallback else PRECISE_YAW
  if abs(yaw)>yaw_limit:raw[2]=math.copysign(min(.10,max(.035,.7*abs(yaw))),yaw)
  xy,w=speed_factors(self.measured);raw[:2]*=xy;raw[2]*=w
  command=np.asarray(cap_sdk(bound(slew_effort(previous,bound(raw),min(.1,max(.001,source_dt)))),'slow'))
  # Do not continue a leftward command while already on the forbidden side.
  if left>0. and command[1]>0.:command[1]=0.
  return Step(tuple(map(float,command)),[],False)

class TaskStopReview:
 def __init__(self,point,arrival_policy=None):
  self.point=point;self.goal=pose(point['pose']);self.checks=0
  self.arrival_policy=(arrival_policy or T2Arrival(self.goal)) if point.get('point_id')=='point_2' else None
 def check(self,mp,stationary):
  if self.arrival_policy is not None:
   verdict=self.arrival_policy.check(mp,stationary);self.checks=self.arrival_policy.checks;return verdict
  self.checks+=1;mp=pose(mp);xy=float(np.linalg.norm(mp[:2]-self.goal[:2]));yaw=abs(wrap(mp[2]-self.goal[2]))
  if stationary and xy<=POSITION and yaw<=YAW:return 'arrived'
  if self.checks>=3:
   if stationary and self.point.get('point_id')=='point_1' and xy<.07 and yaw<=YAW:return 't1_fallback'
   return 'park'
  return 'correct'
