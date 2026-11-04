"""Hypothesis simulation only. No robot libraries, ROS, CAN, or runtime settings."""
from pathlib import Path
import math,json,statistics,csv
R=Path('/home/arx/arx-competition-control');D=R/'reports/upward-damping-offline';D.mkdir(exist_ok=True)
DT=.0005;SUB=5;QSTEP=25/65535;VSTEP=90/4095;TSTEP=36/4095

def simulate(J,Fs,extra):
 h=2.415693904;v=0.;u=0.;records=[];max_added_power=-1e9;max_delta=0.
 for k in range(int(8/DT)):
  t=k*DT;ref=min(5.,2.+.5*t);active=t<6
  if k%SUB==0:
   hm=round(h/QSTEP)*QSTEP;mv=round(-v/VSTEP)*VSTEP
   base=max(-17.,min(13.,2.*(hm-ref)-2.))
   delta=max(-.1,min(.1,-.2*mv)) if extra and active and abs(mv)>.05 else 0.
   assert delta*mv<=1e-12 and abs(delta)<=.10000001
   max_added_power=max(max_added_power,delta*mv);max_delta=max(max_delta,abs(delta))
   tau=max(-17.,min(13.,base+delta));code=int((tau+18.)/36.*4095);u=-(code*TSTEP-18.)
   records.append((t,h,v,ref,-u,delta))
  gravity=1.862457577+.011077261*(h-9.)
  cog=.04032089*math.sin(2*math.pi*h/.393175)+.02962288*math.cos(2*math.pi*h/.393175)
  drive=u-gravity-cog-.01854631*v
  if abs(v)<1e-8 and abs(drive)<=Fs:v=0.
  else:
   friction=.94247714*(1 if (v if abs(v)>1e-8 else drive)>0 else -1)
   nv=v+(drive-friction)/J*DT
   if v*nv<0 and abs(drive)<=Fs:nv=0.
   v=nv;h+=v*DT
 window=[x for x in records if x[0]<6 and 2.47<x[1]<4.55]
 vel=[x[2] for x in window];run=0.;stops=[]
 for x in window:
  if abs(x[2])<.03:run+=DT*SUB
  else:
   if run>.05:stops.append(run)
   run=0.
 mean=statistics.mean(vel) if vel else 0.
 summary={'J_normalized':J,'static_friction_assumed':Fs,'extra_damping':extra,'moving_velocity_cv':statistics.pstdev(vel)/abs(mean) if mean else None,'stops_over50ms':len(stops),'max_stop_s':max(stops,default=0.),'position_final_rad':h,'max_extra_torque_sdk':max_delta,'max_extra_power_proxy':max_added_power,'peak_velocity_rad_s':max(map(abs,vel),default=0.)}
 return summary,records
rows=[]
for J in [.010,.0148,.022]:
 for Fs in [1.0,1.1,1.2]:
  for extra in [False,True]:
   result,records=simulate(J,Fs,extra);rows.append(result)
   if J==.0148 and Fs==1.1:
    with (D/('candidate.csv' if extra else 'baseline.csv')).open('w') as f:
     writer=csv.writer(f);writer.writerow(['time_s','position_rad','velocity_rad_s','reference_rad','torque_sdk','extra_damping_sdk']);writer.writerows(records)
comparisons=[]
for a,b in zip(rows[::2],rows[1::2]):comparisons.append({'J':a['J_normalized'],'Fs':a['static_friction_assumed'],'cv_baseline':a['moving_velocity_cv'],'cv_candidate':b['moving_velocity_cv'],'stops_baseline':a['stops_over50ms'],'stops_candidate':b['stops_over50ms'],'candidate_improves_cv':b['moving_velocity_cv']<a['moving_velocity_cv']})
v={'hypothesis_simulation_only':True,'plant_model_independently_validated':False,'real_motion_improvement_demonstrated':False,'parameters_applied_to_robot':False,'candidate':'upward-reference-only additional dissipative damping; coefficient0.2, cap0.1 normalized SDK torque units; deadband0.05rad/s','unchanged_conditions':['downward motion','homing','steady holding','speed reference','torque envelope'],'scenario_comparisons':comparisons,'scenarios':rows,'warning':'Replay proves candidate bounds and examines assumed friction scenarios, not effectiveness on the real machine.'}
(D/'scenario-results.json').write_text(json.dumps(v,ensure_ascii=False,indent=2));print(json.dumps({'scenarios':len(rows),'cv_improvements':sum(x['candidate_improves_cv'] for x in comparisons),'comparisons':comparisons,'robot_parameters_changed':False},ensure_ascii=False))
