"""Offline identification from previously captured logs. No ROS/CAN imports or writes."""
from pathlib import Path
import json,re,numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear
R=Path('/home/arx/arx-competition-control');D=R/'reports/continuity-comparison'
def prepare(t,h,u):
 t=np.asarray(t);h=np.asarray(h);u=np.asarray(u)
 grid=np.arange(t[0]+.12,t[-1]-.12,.01);hh=np.interp(grid,t,h);uu=savgol_filter(np.interp(grid,t,u),11,3)
 v=savgol_filter(hh,11,3,deriv=1,delta=.01);a=savgol_filter(hh,11,3,deriv=2,delta=.01)
 mask=(abs(v)>.08)&(abs(v)<3)&(hh>1)&(hh<17.95)
 hh,v,a,uu=hh[mask],v[mask],a[mask],uu[mask]
 period=.393175
 cols=[a,v,np.ones_like(v),np.sign(v),hh-9]
 for k in [1,2]:cols.extend([np.sin(2*np.pi*k*hh/period),np.cos(2*np.pi*k*hh/period)])
 return np.array(cols).T,uu,v
sets={}
for n in ['before_reboot','after_reboot']:
 x=np.loadtxt(D/(n+'-reference-reconstruction.csv'),delimiter=',',skiprows=1)
 sets[n]=prepare(x[:,0],x[:,1],-x[:,3])
fb=[];cmd=[]
for l in (R/'reports/continuity-direction-test/can5.log').read_text().splitlines():
 m=re.fullmatch(r'\((\d+\.\d+)\) can5 ([0-9A-F]+)#([0-9A-F]{16})',l)
 if not m:continue
 ts,cid,h=m.groups();ts=float(ts);b=bytes.fromhex(h)
 if cid=='000' and b[0]&15==1:fb.append((ts,25-(int.from_bytes(b[1:3],'big')*25/65535-12.5)))
 if cid=='001' and not h.startswith('FFFFFFFF'):cmd.append((ts,-((((b[6]&15)<<8)|b[7])*36/4095-18)))
f=np.array(fb);c=np.array(cmd);grid=np.arange(max(f[0,0],c[0,0]),min(f[-1,0],c[-1,0]),.01)
sets['direction']=prepare(grid-grid[0],np.interp(grid,f[:,0],f[:,1]),np.interp(grid,c[:,0],c[:,1]))
A=np.vstack([sets['before_reboot'][0],sets['direction'][0]]);y=np.concatenate([sets['before_reboot'][1],sets['direction'][1]]);v=np.concatenate([sets['before_reboot'][2],sets['direction'][2]])
w=np.ones(len(y));w[v<0]=np.sqrt(np.sum(v>0)/max(1,np.sum(v<0)))
# Parameters are normalized by command encoding; these are NOT physical SI gains.
low=[0,0,0,0,-.2,-1,-1,-1,-1];high=[10,10,10,10,.2,1,1,1,1]
fit=lsq_linear(A*w[:,None],y*w,bounds=(low,high),tol=1e-10,max_iter=500)
validation=sets['after_reboot'];rmse=float(np.sqrt(np.mean((validation[0]@fit.x-validation[1])**2)))
summary={'offline_only':True,'hardware_or_runtime_changes':False,'model':'u_sdk = J*a + B_total*v + G + Fc*sign(v) + linear_height + spatial_fourier','parameter_units':'normalized SDK command units, not physical torque units','parameters':dict(zip(['J','B_total','G','Fc','height_slope','sin1','cos1','sin2','cos2'],map(float,fit.x))),'training_moving_samples':len(y),'descending_moving_samples':int(np.sum(v<0)),'ascending_moving_samples':int(np.sum(v>0)),'unweighted_design_condition_number':float(np.linalg.cond(A)),'held_out_ascent_rmse_sdk_units':rmse,'fit_success':bool(fit.success),'inertia_at_lower_constraint':bool(fit.x[0]<1e-5),'suitable_for_automatic_gain_selection':False,'limitations':['Descending data only covers one short high-position interval.','Motor model and mechanism dynamics unconfirmed.','Static and kinetic friction differ; this fit excludes stationary samples.','No candidate is applied to the robot.']}
(D/'offline-response-fit.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));print(json.dumps(summary,ensure_ascii=False))
