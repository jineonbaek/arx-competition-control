"""Recorded-file analysis only; no robot I/O."""
from pathlib import Path
import json,re,numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear
R=Path('/home/arx/arx-competition-control');D=R/'reports/low-bidirectional-diagnostic'
phases=json.loads((D/'phases.json').read_text());fb=[];cmd=[];last=None;circle=-1
for l in (D/'can5.log').read_text().splitlines():
 m=re.fullmatch(r'\((\d+\.\d+)\) can5 ([0-9A-F]+)#([0-9A-F]{16})',l)
 if not m:continue
 ts,cid,h=m.groups();ts=float(ts);b=bytes.fromhex(h)
 if cid=='000' and b[0]&15==1:
  q=int.from_bytes(b[1:3],'big')*25/65535-12.5
  if last is not None:
   if q-last>12.5:circle-=1
   elif q-last< -12.5:circle+=1
  last=q;fb.append((ts,-(q+25*circle),((b[3]<<4)|(b[4]>>4))*90/4095-45,b[0]>>4,b[7]))
 if cid=='001' and not h.startswith('FFFFFFFF'):cmd.append((ts,-((((b[6]&15)<<8)|b[7])*36/4095-18)))
f=np.array(fb);c=np.array(cmd)
def dataset(t,h,u):
 grid=np.arange(t[0]+.12,t[-1]-.12,.01);hh=np.interp(grid,t,h);uu=savgol_filter(np.interp(grid,t,u),11,3);v=savgol_filter(hh,11,3,deriv=1,delta=.01);a=savgol_filter(hh,11,3,deriv=2,delta=.01)
 mask=(abs(v)>.08)&(abs(v)<3)&(hh>1)&(hh<17.95);hh,v,a,uu=hh[mask],v[mask],a[mask],uu[mask]
 cols=[a,v,np.ones_like(v),np.sign(v),hh-9]
 for k in [1,2]:cols += [np.sin(2*np.pi*k*hh/.393175),np.cos(2*np.pi*k*hh/.393175)]
 return np.array(cols).T,uu,v
sets=[];summaries=[]
for i,p in enumerate(phases):
 mask=(f[:,0]>=p['command_time'])&(f[:,0]<=p['finished_time']);x=f[mask];u=np.interp(x[:,0],c[:,0],c[:,1]);sets.append(dataset(x[:,0],x[:,1],u))
 # Exact-position plateaus, excluding start and settled end via height range.
 lo,hi=min(x[0,1],x[-1,1]),max(x[0,1],x[-1,1]);moving=x[(x[:,1]>lo+.02)&(x[:,1]<hi-.02)];plateaus=[]
 if len(moving):
  a=last=moving[0]
  for row in moving[1:]:
   if row[1]!=last[1]:
    dur=last[0]-a[0]
    if dur>.05:plateaus.append(float(dur))
    a=row
   last=row
 summaries.append({'phase':i,'target_rad':p['target_rad'],'settled_angle_rad':float(np.median(x[x[:,0]>x[-1,0]-1,1])),'range_rad':[float(min(x[:,1])),float(max(x[:,1]))],'plateaus_over50ms':len(plateaus),'longest_plateau_s':max(plateaus,default=0),'moving_fit_samples':len(sets[-1][1]),'direction':'up' if p['target_rad']>p['previous_target_rad'] else 'down'})
a=np.loadtxt(R/'reports/continuity-comparison/before_reboot-reference-reconstruction.csv',delimiter=',',skiprows=1);old=dataset(a[:,0],a[:,1],-a[:,3]);b=np.loadtxt(R/'reports/continuity-comparison/after_reboot-reference-reconstruction.csv',delimiter=',',skiprows=1);held_up=dataset(b[:,0],b[:,1],-b[:,3])
A=np.vstack([old[0],sets[0][0],sets[1][0]]);y=np.concatenate([old[1],sets[0][1],sets[1][1]]);v=np.concatenate([old[2],sets[0][2],sets[1][2]])
fit=lsq_linear(A,y,bounds=([0,0,0,0,-.2,-1,-1,-1,-1],[10,10,10,10,.2,1,1,1,1]),tol=1e-10,max_iter=500)
def rmse(s):return float(np.sqrt(np.mean((s[0]@fit.x-s[1])**2)))
report={'phases':summaries,'normalized_model_parameters':dict(zip(['J','B_total','G','Fc','height_slope','sin1','cos1','sin2','cos2'],map(float,fit.x))),'training_ascending_moving_samples':int(sum(v>0)),'training_descending_moving_samples':int(sum(v<0)),'held_out_ascent_rmse_sdk_units':rmse(held_up),'held_out_low_descent_rmse_sdk_units':rmse(sets[2]),'damping_hits_lower_bound':bool(fit.x[1]<1e-5),'model_suitable_for_automatic_gain_selection':False,'parameters_applied':False,'reason':'Model is descriptive; friction and damping separation and controller changes need further validation.'}
(D/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
