"""Pure low-effort straight-chain control. Deliberately no maps or transport."""
from dataclasses import dataclass
import math
import numpy as np
from mppi_trial.effort_budget import bound
from mppi_trial.session_policy import slew_effort
from mppi_trial.trial_policy import speed_factors

RADIUS=.10
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def pose(v):
 a=np.asarray(v,dtype=float)
 if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError('special_invalid_pose')
 return a
def body_delta(a,b):
 d=b[:2]-a[:2];c,s=math.cos(a[2]),math.sin(a[2])
 return np.array([c*d[0]+s*d[1],-s*d[0]+c*d[1],wrap(b[2]-a[2])])
@dataclass
class Step:
 command:tuple
 passed_ids:list
 terminal_near:bool

class StraightChain:
 def __init__(self,entry_map,entry_odom,points):
  self.entry_map=pose(entry_map);self.entry_odom=pose(entry_odom)
  if not points or len(points)>200:raise ValueError('special_empty_or_long_chain')
  self.points=points;self.index=0;self.radius=RADIUS;self.previous_sample=None;self.measured=np.zeros(3)
  angle=self.entry_odom[2]-self.entry_map[2];c,s=math.cos(angle),math.sin(angle)
  self.rotation=np.array([[c,-s],[s,c]])
  self.goals=[self.rotation@(pose(p['pose'])[:2]-self.entry_map[:2])+self.entry_odom[:2] for p in points]
  self.start=self.entry_odom[:2].copy();self.yaw=self.entry_odom[2]
 @property
 def target(self):return self.goals[self.index]
 def sample(self,m,o,stamp,now):
  m,o=pose(m),pose(o)
  if not all(math.isfinite(v) for v in (stamp,now)) or not 0<=now-stamp<=.65:raise ValueError('special_pose_stale')
  old=self.previous_sample
  if old:
   t,pm,po=old;dt=stamp-t
   if dt<=0:raise ValueError('special_source_repeated')
   if dt>.4:raise ValueError('special_odometry_gap')
   dm,do=body_delta(pm,m),body_delta(po,o)
   if max(np.linalg.norm(dm[:2]),np.linalg.norm(do[:2]))>max(.06,.45*dt) or np.linalg.norm(dm[:2]-do[:2])>.04 or max(abs(dm[2]),abs(do[2]))>max(.08,.6*dt) or abs(wrap(dm[2]-do[2]))>.04:raise ValueError('special_localization_jump')
   self.measured=do/dt
  self.previous_sample=(stamp,m.copy(),o.copy());return m,o
 def step(self,map_pose,odom_pose,stamp,now,dt,previous):
  m,o=self.sample(map_pose,odom_pose,stamp,now);previous=pose(previous)
  if not math.isfinite(dt) or not 0<dt<=.3:raise ValueError('special_cycle_overrun')
  passed=[]
  if self.index<len(self.goals)-1 and np.linalg.norm(self.target-o[:2])<=RADIUS:
   passed=[self.points[self.index]['id']];self.start=o[:2].copy();self.index+=1
  error=float(np.linalg.norm(self.target-o[:2]));terminal=self.index==len(self.goals)-1 and error<=RADIUS
  if terminal:return Step((0.,0.,0.),passed,True)
  line=self.target-self.start;length=float(np.linalg.norm(line));unit=line/max(length,1e-12)
  along=float(np.clip((o[:2]-self.start)@unit,0.,length));look=self.start+unit*min(length,along+.15)
  direction=look-o[:2];norm=float(np.linalg.norm(direction))
  if norm<1e-9:direction=self.target-o[:2];norm=float(np.linalg.norm(direction))
  c,s=math.cos(o[2]),math.sin(o[2]);body=np.array([[c,s],[-s,c]])@direction/max(norm,1e-12)
  # Fixed commissioned effort, never a guessed SDK↔SI conversion.
  effort=min(.22,max(.14,1.4*max(0.,error-RADIUS)))
  yaw=wrap(self.yaw-o[2]);w=0. if abs(yaw)<.035 else float(np.clip(.6*yaw,-.10,.10))
  raw=np.r_[body*effort,w];xy_factor,w_factor=speed_factors(self.measured);raw[:2]*=xy_factor;raw[2]*=w_factor
  command=bound(slew_effort(previous,bound(raw),dt))
  return Step(tuple(map(float,command)),passed,False)
