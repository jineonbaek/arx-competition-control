"""Pure low-effort straight-chain control. Deliberately no maps or transport."""
from dataclasses import dataclass
from chassis_pose_policy import MOTION_MAX_AGE_NS
import math
import numpy as np
from mppi_trial.effort_budget import bound
from mppi_trial.session_policy import slew_effort
from mppi_trial.speed_profile import NORMAL_SDK,SLOW_SDK

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
 def __init__(self,entry_map,entry_odom,points,*,continuous=False,terminal_precision=True):
  self.continuous=continuous;self.terminal_precision=terminal_precision
  self.entry_map=pose(entry_map);self.entry_odom=pose(entry_odom)
  if not points or len(points)>200:raise ValueError('special_empty_or_long_chain')
  self.points=points;self.index=0;self.radius=RADIUS;self.previous_sample=None;self.measured=np.zeros(3)
  angle=self.entry_odom[2]-self.entry_map[2];c,s=math.cos(angle),math.sin(angle)
  self.rotation=np.array([[c,-s],[s,c]])
  self.goals=[self.rotation@(pose(p['pose'])[:2]-self.entry_map[:2])+self.entry_odom[:2] for p in points]
  self.start=self.entry_odom[:2].copy();self.yaw=self.entry_odom[2]
  self.filtered_motion=np.zeros(2);self.terminal_correction=False
  self.reset_start_assist()
 def reset_start_assist(self):
  self.start_assist_floor=0.;self.start_assist_origin=None
 def request_start_assist(self):
  if self.previous_sample is None:raise ValueError('start_assist_requires_fresh_source')
  self.start_assist_floor=.3 if self.start_assist_floor<.3 else .6
  self.start_assist_origin=self.previous_sample[2][:2].copy()
  return self.start_assist_floor
 @property
 def target(self):return self.goals[self.index]
 def sample(self,m,o,stamp,now):
  m,o=pose(m),pose(o)
  if not all(math.isfinite(v) for v in (stamp,now)) or not 0<=now-stamp<=MOTION_MAX_AGE_NS/1e9:raise ValueError('special_pose_stale')
  old=self.previous_sample
  if old:
   t,pm,po=old;dt=stamp-t
   if dt<=0:raise ValueError('special_source_repeated')
   if dt>MOTION_MAX_AGE_NS/1e9:raise ValueError('special_odometry_gap')
   dm,do=body_delta(pm,m),body_delta(po,o)
   if max(np.linalg.norm(dm[:2]),np.linalg.norm(do[:2]))>max(.06,.90*dt) or np.linalg.norm(dm[:2]-do[:2])>.04 or max(abs(dm[2]),abs(do[2]))>max(.08,1.2*dt) or abs(wrap(dm[2]-do[2]))>.04:raise ValueError('special_localization_jump')
   self.measured=do/dt
  self.previous_sample=(stamp,m.copy(),o.copy());return m,o
 def step(self,map_pose,odom_pose,stamp,now,dt,previous):
  old_pose=None if self.previous_sample is None else self.previous_sample[2].copy()
  source_dt=dt if self.previous_sample is None else stamp-self.previous_sample[0]
  m,o=self.sample(map_pose,odom_pose,stamp,now);previous=pose(previous)
  if not math.isfinite(dt) or not 0<dt<=.3:raise ValueError('special_cycle_overrun')
  control_dt=min(.1,max(.001,source_dt))
  c,s=math.cos(o[2]),math.sin(o[2]);to_world=np.array([[c,-s],[s,c]])
  motion=to_world@self.measured[:2]
  self.filtered_motion+=(1.-math.exp(-control_dt/.20))*(motion-self.filtered_motion)
  if self.start_assist_origin is not None and np.linalg.norm(o[:2]-self.start_assist_origin)>=.015:
   self.reset_start_assist()
  passed=[]
  def crossed(target):
   if old_pose is None:return False
   delta=o[:2]-old_pose[:2];square=float(delta@delta)
   if square<1e-18:return False
   fraction=float(np.clip((target-old_pose[:2])@delta/square,0.,1.))
   return np.linalg.norm(old_pose[:2]+fraction*delta-target)<=RADIUS
  if self.index<len(self.goals)-1 and (np.linalg.norm(self.target-o[:2])<=RADIUS or self.continuous and crossed(self.target)):
   passed=[self.points[self.index]['id']];self.start=o[:2].copy();self.index+=1;self.reset_start_assist()
  error=float(np.linalg.norm(self.target-o[:2]));final=self.index==len(self.goals)-1
  precise=self.continuous and self.terminal_precision and final
  arrival_radius=.045 if precise else RADIUS
  goal_yaw=wrap(self.points[-1]['pose'][2]+self.entry_odom[2]-self.entry_map[2]) if precise else self.yaw
  terminal=final and error<=arrival_radius and (not precise or abs(wrap(goal_yaw-o[2]))<=.035)
  if precise and error<.5:self.reset_start_assist()
  if terminal and not self.continuous:
   self.reset_start_assist();return Step((0.,0.,0.),passed,True)
  line=self.target-self.start;length=float(np.linalg.norm(line));unit=line/max(length,1e-12)
  # Increase preview with measured speed and source latency, never a forged pose.
  speed=max(float(np.linalg.norm(motion)),float(np.linalg.norm(self.filtered_motion)))
  age=max(0.,now-stamp)
  lookahead=min(.65,.15+speed*(age+.40))
  along=float(np.clip((o[:2]-self.start)@unit,0.,length));look=self.start+unit*min(length,along+lookahead)
  direction=look-o[:2];norm=float(np.linalg.norm(direction))
  if norm<1e-9:direction=self.target-o[:2];norm=float(np.linalg.norm(direction))
  if terminal and self.continuous:direction=unit;norm=float(np.linalg.norm(unit))
  # Collinear pass points retain cruise; finals and significant bends slow early.
  effort=NORMAL_SDK
  final=self.index==len(self.goals)-1
  bend=False
  if not final:
   outgoing=self.goals[self.index+1]-self.target
   outnorm=float(np.linalg.norm(outgoing))
   bend=outnorm<1e-9 or float(unit@(outgoing/max(outnorm,1e-12)))<math.cos(.25)
  if (final and self.terminal_precision) or (bend and not self.continuous):
   closing=max(0.,float(motion@unit),float(self.filtered_motion@unit))
   remaining=max(0.,error-arrival_radius-closing*(age+.20))
   floor=SLOW_SDK
   effort=min(effort,max(min(floor,NORMAL_SDK),1.2*remaining))
  if self.terminal_correction:effort=min(effort,SLOW_SDK)
  # Only the normal-special runtime requests this floor. Task approaches and
  # test backoff never request it. Retain slew limits and the .60 total cap.
  effort=min(NORMAL_SDK,max(effort,self.start_assist_floor))
  desired=direction/max(norm,1e-12)*effort
  normal=np.array([-unit[1],unit[0]])
  lateral=float(self.filtered_motion@normal)
  desired-=normal*float(np.clip(.6*lateral,-.25*effort,.25*effort))
  desired*=min(1.,effort/max(float(np.linalg.norm(desired)),1e-12))
  body=np.array([[c,s],[-s,c]])@desired
  if precise and error<=arrival_radius and not terminal:body[:]=0.
  yaw=wrap(goal_yaw-o[2]);w=0. if abs(yaw)<.035 else float(np.clip(.6*yaw,-NORMAL_SDK,NORMAL_SDK))
  if precise and abs(yaw)>=.035:w=math.copysign(min(.10,max(.035,.7*abs(yaw))),yaw)
  raw=np.r_[body,w]
  command=bound(slew_effort(previous,bound(raw),control_dt))
  return Step(tuple(map(float,command)),passed,terminal)
