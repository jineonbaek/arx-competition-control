"""Explicit empty-arm sequential execution through the unchanged web API.
Never sends a gripper target, enables hardware, or loads a driver.
"""
import argparse,json,time,http.cookiejar,urllib.request
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from control import arm_model
from fast_visual_level import empty_prepare_geometry,table_front_x
from agent.cabinet_loaded_sweep import _boxes,_root

def main():
 p=argparse.ArgumentParser();p.add_argument('--report',required=True);p.add_argument('--frame',required=True);p.add_argument('--execute',action='store_true');p.add_argument('--resume-after',type=int,default=0);a=p.parse_args()
 report=json.loads(Path(a.report).read_text());plan=report['coarse'];points=np.asarray(plan['native_plan']['joint_waypoints_rad'],float)
 frame=dict(np.load(a.frame,allow_pickle=False));base='http://127.0.0.1:9983'
 op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()));op.open(base+'/',timeout=3).read()
 def state():return json.load(op.open(base+'/api/state',timeout=2))
 def send(data):
  if data['type'] not in ('arm.joints','arm.stop') or data['arm']!='right':raise ValueError('Only right arm positions/hold allowed')
  req=urllib.request.Request(base+'/api/command',data=json.dumps(data).encode(),headers={'Origin':base,'Content-Type':'application/json'})
  r=json.load(op.open(req,timeout=3))
  if r.get('ok') is not True:raise ValueError('Command not acknowledged')
  return r
 initial=state();h=initial['body']['height'];left=np.asarray(initial['arms']['left']['joints']);start=np.asarray(initial['arms']['right']['joints'])
 clamp=initial['arms']['right']['gripper'];fixed={k:initial['body'][k] for k in ('waist','head_yaw','head_pitch')}
 model=arm_model('right');lm=arm_model('left')
 expected_epoch=initial['command_epoch'];records=[];attempted=False
 def guard(s):
  if not all(s['arms'][k]['fresh'] for k in ('left','right')) or not s['body']['fresh']:raise ValueError('Feedback stale')
  if s['motion_reserved'] or s['competition_reserved'] or s['chassis_active']:raise ValueError('Another task owns motion')
  if s['command_epoch']!=expected_epoch:raise ValueError('Other command changed state')
  if max(abs(np.asarray(s['arms']['left']['joints'])-left))>np.radians(.5):raise ValueError('Left arm changed')
  if abs(s['body']['height']-h)>.05 or any(abs(s['body'][k]-v)>.02 for k,v in fixed.items()):raise ValueError('Body/camera moved')
  if abs(s['arms']['right']['gripper']-clamp)>.05:raise ValueError('Gripper feedback changed')
 guard(initial)
 if points.ndim!=2 or points.shape[1]!=6 or not 2<=len(points)<=100 or not np.isfinite(points).all():raise ValueError('Invalid path')
 if a.resume_after:
  if not 0<a.resume_after<len(points)-1:raise ValueError('Invalid resume index')
  old_result=json.loads(Path('empty-approach-result.json').read_text())
  if old_result['status']!='held' or old_result['records'][-1]['segment']!=a.resume_after:raise ValueError('Resume must match verified completed checkpoint')
  # The failed next target is retained, not skipped. Recheck the entire suffix
  # from the measured held pose before dispatch.
  if max(abs(points[a.resume_after+1]-start))>np.radians(2.):raise ValueError('Held state too far from next checkpoint')
  points=np.vstack([start,points[a.resume_after+1:]])
 elif max(abs(points[0]-start))>np.radians(.5):raise ValueError('Path start changed')
 if not 0<=time.time()-float(frame['image_time'])<=60:raise ValueError('Scene stale')
 if max(abs(np.asarray(frame['joints'])-start))>np.radians(.5):raise ValueError('Frame state changed')
 # Fresh observed cabinet plane plus a measured table plane; no carried object.
 from agent.cabinet_preview_geometry import localize,opening
 root=Path('/home/arx/arx-place-hold-20261116')
 slot=opening(json.loads((root/'config/cabinet-profiles.json').read_text()),'demo')
 window=localize(frame,profile=slot,now=float(frame['image_time']))
 if np.linalg.norm(np.asarray(window['center_base_m'])-plan['window']['center_base_m'])>.005:raise ValueError('Cabinet changed since planning')
 center=np.asarray(window['center_base_m']);out=np.asarray(window['outward_normal_base']);table=table_front_x(frame,details=True)
 normal=np.asarray(table['front_normal_base']);base_root=_root(model,h)
 def sweep(q0,q1):
  empty_prepare_geometry(model,lm,left,q0,q1,h)
  count=max(2,int(np.ceil(np.max(abs(q1-q0))/np.radians(.25)))+1)
  bound=model.reach*sum(abs(q1-q0))/(2*(count-1))+.003
  for q in np.linspace(q0,q1,count):
   for box in _boxes(model,q,h,base_root):
    if float(((box-center)@out).min())-bound<.02:raise ValueError('Empty arm too close to cabinet plane')
    gap=max(0.,float(table['front_offset_m']-(box@normal).max()),float(box[:,2].min()-table['top_m']),float(table['bottom_m']-box[:,2].max()))
    if gap-bound<.005:raise ValueError('Empty arm/table clearance unavailable')
 points[0]=start
 for q0,q1 in zip(points,points[1:]):sweep(q0,q1)
 print(json.dumps(dict(stage='checked_empty_path',segments=len(points)-1,execute=a.execute,gripper_commands=0)),flush=True)
 if not a.execute:return
 try:
  overall=time.monotonic()
  for index,target in enumerate(points[1:],1):
   s=state();guard(s);q=np.asarray(s['arms']['right']['joints'])
   if time.monotonic()-overall>180:raise ValueError('Overall timeout')
   sweep(q,target);guard(state())
   attempted=True;send(dict(type='arm.joints',arm='right',joints=np.degrees(target).tolist()));expected_epoch+=1
   deadline=time.monotonic()+12;history=[]
   while time.monotonic()<deadline:
    s=state();guard(s);row=s['arms']['right'];q=np.asarray(row['joints']);error=max(abs(q-target));stamp=row['timestamp']
    if not history or stamp>history[-1][0]:history.append((stamp,q.copy()))
    history=[v for v in history if stamp-v[0]<.4]
    actual_pose=model.wrist_transform(q,h);target_pose=model.wrist_transform(target,h)
    position_error_mm=float(np.linalg.norm(actual_pose[:3,3]-target_pose[:3,3])*1000)
    rotation_error_deg=float(np.degrees(Rotation.from_matrix(actual_pose[:3,:3].T@target_pose[:3,:3]).magnitude()))
    if error<np.radians(1.5) and position_error_mm<=5. and rotation_error_deg<=2. and len(history)>2 and history[-1][0]-history[0][0]>.15 and np.max(np.ptp([v[1] for v in history],axis=0))<.003:break
    time.sleep(.08)
   else:raise ValueError('Joint arrival timeout')
   records.append(dict(segment=index+a.resume_after,error_deg=float(np.degrees(error)),position_error_mm=position_error_mm,rotation_error_deg=rotation_error_deg,q=q.tolist()))
   print(json.dumps(records[-1]),flush=True)
  result=dict(status='outside_arrived',records=records,state=state(),gripper_commands=0)
 except Exception as exc:
  result=dict(status='held',reason=str(exc),records=records,gripper_commands=0)
  if attempted:
   try:result['stop']=send(dict(type='arm.stop',arm='right'))
   except Exception as stop:result['stop_error']=str(stop)
 Path('empty-approach-result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
 print(json.dumps({k:v for k,v in result.items() if k not in ('state','records')},ensure_ascii=False),flush=True)

if __name__=='__main__':main()
