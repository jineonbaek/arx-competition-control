"""Illustrative delayed first-order plant, based on recorded .6 speed and coast.
Not a physical calibration or a claim of hardware validation.
"""
import importlib.util,sys,math,json
from pathlib import Path
import numpy as np
from mppi_trial.stationary import StopWindow

def module(path,name):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m

def simulate(m,*,cap=.6,delay=.25,tau=.20,noise=.01,new=False):
 m.NORMAL_SDK=cap;c=m.StraightChain([0,.04,0],[0,.04,0],[dict(id='goal',pose=[5.,0.,0.])])
 dt=.01;p=np.array([0.,.04,0.]);v=np.zeros(2);cmd=np.zeros(3);history=[p.copy()];vel_history=[v.copy()];stop=None;arrived=False;max_x=0.;side=[];turns=0;last_sign=0;max_command=0.;last_source=None
 for i in range(1,3001):
  t=i*dt;v+=(.80*cmd[:2]-v)*min(1.,dt/tau);p[:2]+=v*dt;history.append(p.copy());vel_history.append(v.copy());max_x=max(max_x,p[0])
  if 1.<p[0]<3.:side.append(float(p[1]-.04*(1.-p[0]/5.)))
  if i%10:continue
  index=max(0,i-round(delay/dt));source=index*dt;sample=history[index].copy();sample[1]+=(.001+noise*min(1.,float(np.linalg.norm(vel_history[index]))/.05))*math.sin(2*math.pi*source/.6)
  if source==last_source:continue
  last_source=source
  step=c.step(sample,sample,source,t,.1,cmd)
  if step.terminal_near and stop is None:stop=StopWindow(after=t)
  if stop is not None:
   cmd=np.zeros(3)
   if not new and np.linalg.norm(sample[:2]-c.target)>.10:stop=None
   elif stop.add(source,sample):
    if np.linalg.norm(sample[:2]-c.target)>.10:c.terminal_correction=True;stop=None
    else:arrived=True;break
  else:cmd=np.asarray(step.command)
  sign=0 if abs(cmd[0])<.015 else (1 if cmd[0]>0 else -1)
  if sign and last_sign and sign!=last_sign:turns+=1
  if sign:last_sign=sign
  max_command=max(max_command,float(np.linalg.norm(cmd[:2])))
 return dict(cap=cap,delay=delay,tau=tau,arrived=arrived,elapsed=round(t,2),max_overshoot=round(max(0.,max_x-5),4),final_error=round(float(np.linalg.norm(p[:2]-[5.,0.])),4),straight_lateral_rms=round(float(np.sqrt(np.mean(np.square(side)))) if side else 0.,4),reversals=turns,max_sdk=round(max_command,3))
