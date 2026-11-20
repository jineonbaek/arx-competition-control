"""Reusable D435 loaded coarse correction, unchanged driver, no gripper commands.
Default plans only. --execute sends bounded position segments, never insertion.
"""
import argparse,json,time,urllib.request,http.cookiejar
from pathlib import Path
import numpy as np

def arm_command(q):
 q=np.asarray(q,float)
 if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('Invalid joints')
 return dict(type='arm.joints',arm='right',joints=np.degrees(q).tolist())

def validate_start(s,f,points,now):
 q=np.asarray(s['arms']['right']['joints']);p=np.asarray(points,float)
 if not s['execution_enabled'] or not s['body']['fresh'] or not all(s['arms'][a]['fresh'] for a in ('left','right')):raise ValueError('Feedback unavailable')
 if any(s[k] for k in ('motion_reserved','competition_reserved','chassis_active')):raise ValueError('Other controller active')
 if not 0<=now-float(f['image_time'])<=10:raise ValueError('D435 frame stale')
 if abs(s['body']['height']-float(f['lift_rad']))>.03:raise ValueError('Capture height changed')
 if p.ndim!=2 or p.shape[1]!=6 or len(p)<2 or not np.isfinite(p).all():raise ValueError('Invalid plan')
 if max(abs(q-p[0]))>np.radians(.5) or max(abs(q-np.asarray(f['joints'])))>np.radians(.5):raise ValueError('Capture/start pose changed')

def vertical_target(pose,up_mm):
 if not np.isfinite(up_mm) or not 0<up_mm<=5:raise ValueError('Vertical adjustment must be within 5 mm')
 target=np.array(pose,float,copy=True)
 if target.shape!=(4,4) or not np.isfinite(target).all():raise ValueError('Invalid pose')
 target[2,3]+=up_mm/1000.
 return target

def lateral_target(pose,delta_y_mm):
 if not np.isfinite(delta_y_mm) or not 0<abs(delta_y_mm)<=5:raise ValueError('Lateral adjustment must be nonzero and within 5 mm')
 target=np.array(pose,float,copy=True)
 if target.shape!=(4,4) or not np.isfinite(target).all():raise ValueError('Invalid pose')
 target[1,3]+=delta_y_mm/1000.
 return target

def camera_roll_target(pose,camera,roll_deg):
 from scipy.spatial.transform import Rotation
 t=np.array(pose,float,copy=True);c=np.asarray(camera,float)
 if t.shape!=(4,4) or c.shape!=(4,4) or not np.isfinite(np.r_[t.ravel(),c.ravel(),roll_deg]).all() or abs(roll_deg)>1.5:raise ValueError('Invalid bounded view rotation')
 if not np.allclose(c[:3,:3].T@c[:3,:3],np.eye(3),atol=1e-5):raise ValueError('Invalid camera rotation')
 t[:3,:3]=Rotation.from_rotvec(c[:3,2]*np.radians(roll_deg)).as_matrix()@t[:3,:3]
 return t

def roll_up_target(pose,camera,roll_deg,up_mm):
 return vertical_target(camera_roll_target(pose,camera,roll_deg),up_mm)

def axis_step_target(pose,axis,step_deg):
 from scipy.spatial.transform import Rotation
 t=np.array(pose,float,copy=True);a=np.asarray(axis,float)
 if (t.shape!=(4,4) or a.shape!=(3,) or not np.isfinite(np.r_[t.ravel(),a,step_deg]).all()
     or not 0<step_deg<=2.5 or abs(a[2])>1e-9 or abs(np.linalg.norm(a)-1)>1e-6):raise ValueError('Invalid bounded axis correction')
 x=t[:3,0];angle=float(np.arccos(np.clip(x@a,-1.,1.)))
 if angle<=np.radians(step_deg):raise ValueError('Axis correction step exceeds measured discrepancy')
 turn=np.cross(x,a);turn/=np.linalg.norm(turn)
 t[:3,:3]=Rotation.from_rotvec(turn*np.radians(step_deg)).as_matrix()@t[:3,:3]
 return t

def joint6_goal(q,step,hold=None):
 q=np.asarray(q,float);h=q if hold is None else np.asarray(hold,float)
 if q.shape!=(6,) or h.shape!=(6,) or not np.isfinite(np.r_[q,h,step]).all() or not 0<abs(step)<=2:raise ValueError('Invalid J6 correction')
 if np.max(abs(q[:5]-h[:5]))>np.radians(1.5):raise ValueError('Retained reference changed too far')
 goal=h.copy();goal[5]=q[5]+np.radians(step);return goal

def main():
 from scipy.spatial.transform import Rotation
 from control import arm_model
 from fast_visual_level import table_front_x
 from agent.cabinet_loaded_sweep import check_outside_sweep
 from agent.cabinet_preview_geometry import opening
 from agent.visual_battery_level import motion_limits
 from vision_path_bridge import coarse
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');p.add_argument('--max-segments',type=int,default=2);p.add_argument('--stand-off-mm',type=float,default=60.);p.add_argument('--up-mm',type=float);p.add_argument('--delta-y-mm',type=float);p.add_argument('--axis-step-deg',type=float);p.add_argument('--align-top',action='store_true');p.add_argument('--roll-step-deg',type=float);p.add_argument('--compensate-up-mm',type=float);p.add_argument('--j6-step-deg',type=float);p.add_argument('--hold-joints');a=p.parse_args()
 if a.j6_step_deg is not None and (not np.isfinite(a.j6_step_deg) or not 0<abs(a.j6_step_deg)<=2.):raise ValueError('J6 probe limited to 2 degrees')
 if a.compensate_up_mm is not None and (not a.align_top or not np.isfinite(a.compensate_up_mm) or not 0<a.compensate_up_mm<=5):raise ValueError('Roll compensation requires --align-top and 0..5mm up')
 if a.roll_step_deg is not None and (not a.align_top or not np.isfinite(a.roll_step_deg) or not 0<a.roll_step_deg<=1.5):raise ValueError('Explicit roll step requires --align-top and 0..1.5deg')
 if sum((a.up_mm is not None,a.delta_y_mm is not None,a.axis_step_deg is not None,a.align_top or a.j6_step_deg is not None))>1:raise ValueError('Choose one correction mode')
 if not 55<=a.stand_off_mm<=100:raise ValueError('Outside handoff distance must match current loaded sweep policy')
 if not 1<=a.max_segments<=4:raise ValueError('Use 1 to 4 bounded segments')
 root=Path('/home/arx/arx-place-hold-20261116');read=lambda p:json.loads(Path(p).read_text());base='http://127.0.0.1:9983'
 op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()));op.open(base+'/',timeout=3).read()
 def state():return json.load(op.open(base+'/api/state',timeout=2))
 def send(data):
  if data.get('arm')!='right' or data.get('type') not in ('arm.joints','arm.stop') or set(data)-{'arm','type','joints'}:raise ValueError('Command outside scope')
  r=json.load(op.open(urllib.request.Request(base+'/api/command',data=json.dumps(data).encode(),headers={'Origin':base,'Content-Type':'application/json'}),timeout=3))
  if r.get('ok') is not True:raise ValueError('Command not acknowledged')
  return r
 capbase='http://127.0.0.1:9987';cap=json.load(urllib.request.urlopen(urllib.request.Request(capbase+'/api/capture',data=b'',headers={'Origin':capbase}),timeout=45));folder=Path(cap['capture_directory'])
 f=dict(np.load(folder/'d435.npz',allow_pickle=False));s=state();q0=np.array(s['arms']['right']['joints']);h=s['body']['height'];left=np.array(s['arms']['left']['joints']);g=s['arms']['right']['gripper'];epoch=s['command_epoch'];fixed={k:s['body'][k] for k in ('waist','head_yaw','head_pitch')}
 validate_start(s,f,[q0,q0],time.time())
 cfg=read('/home/arx/arx-competition-control/web/config/control.json');m=arm_model('right');lm=arm_model('left');tcp=np.array(read(root/'config/right-grip-tcp.json')['T_wrist_from_tool']);slot=opening(read(root/'config/cabinet-profiles.json'),'demo');battery=read(root/'config/battery-envelope.json');contact=read(root/'web/public/cad/battery-grip.json')['points_object_mm']['tip'];battery['contact_mm']=contact
 if a.axis_step_deg is not None:
  from agent.cabinet_preview_geometry import localize
  from agent.grasp_path import plan_correction
  from vision_path_bridge import native_plan
  window=localize(f,profile=slot,now=float(f['image_time']));axis=-np.asarray(window['outward_normal_base'],float);axis[2]=0.;axis/=np.linalg.norm(axis)
  target=axis_step_target(m.wrist_transform(q0,h)@tcp,axis,a.axis_step_deg)
  path=plan_correction(m,q0,h,tcp,target,cfg['arms']['right']['joint_limits_deg'],timing_limits=motion_limits(read(root/'config/battery-execution.json'),cfg),translation_step_mm=5.,rotation_step_deg=1.)
  proposal=dict(stage='insertion_axis_adjustment',native_plan=native_plan(path),window=window,target_base_from_tool=target.tolist(),gripper_action='retain',axis_step_deg=a.axis_step_deg,insertion_axis_base=axis.tolist())
 elif a.align_top or a.j6_step_deg is not None:
  import sys
  sys.path.insert(0,str(root/'experiments/live-overlays-20261116'))
  from battery_edges import detect_face
  from rim_edges import detect_rim,line_angle_error
  from agent.cabinet_preview_geometry import localize
  from agent.grasp_path import plan_correction
  from vision_path_bridge import native_plan
  f405=dict(np.load(folder/'d405.npz',allow_pickle=False));validate_start(s,f405,[q0,q0],time.time())
  body=detect_face(f405['rgb']);rim=detect_rim(f405['rgb'])
  if body is None or rim is None:raise ValueError('Fresh line pair unavailable')
  angle=line_angle_error(body['quad_px'][:2],rim['edges']['top']['segment_px'])
  if abs(angle)>8:raise ValueError('Image line discrepancy exceeds bounded correction range')
  if abs(angle)<.75:
   print(json.dumps(dict(status='image_edges_parallel',angle_error_deg=angle,gripper_commands=0,robot_motion=False,capture=str(folder))),flush=True);return
  roll_limit=1. if a.roll_step_deg is None else a.roll_step_deg
  roll=float(np.clip(-angle,-roll_limit,roll_limit))
  if a.j6_step_deg is not None:
   goal=joint6_goal(q0,a.j6_step_deg,json.loads(a.hold_joints) if a.hold_joints else None);lims=np.radians(np.array(cfg['arms']['right']['joint_limits_deg']))
   if np.any(goal<lims[:,0]) or np.any(goal>lims[:,1]):raise ValueError('Probe outside joint limits')
   target=m.wrist_transform(goal,h)@tcp
   if np.linalg.norm(target[:3,3]-(m.wrist_transform(q0,h)@tcp)[:3,3])>.004:raise ValueError('Probe TCP shift exceeds 4mm')
   path=dict(joint_waypoints_rad=[q0.tolist(),goal.tolist()],timing=dict(time_from_start_s=[0.,1.]))
  else:
   target=(roll_up_target(m.wrist_transform(q0,h)@tcp,f405['base_from_color'],roll,a.compensate_up_mm)
           if a.compensate_up_mm is not None else camera_roll_target(m.wrist_transform(q0,h)@tcp,f405['base_from_color'],roll))
   path=plan_correction(m,q0,h,tcp,target,cfg['arms']['right']['joint_limits_deg'],timing_limits=motion_limits(read(root/'config/battery-execution.json'),cfg),translation_step_mm=5.,rotation_step_deg=2.)
   proposal=dict(stage='image_edge_parallel',native_plan=native_plan(path),window=localize(f,profile=slot,now=float(f['image_time'])),target_base_from_tool=target.tolist(),gripper_action='retain',image_error_before_deg=angle,camera_roll_deg=roll if a.j6_step_deg is None else None,j6_step_deg=a.j6_step_deg,compensate_up_mm=a.compensate_up_mm)
 elif a.up_mm is None and a.delta_y_mm is None:
  proposal=coarse(f,m,tcp,slot,battery,contact,cfg['arms']['right']['joint_limits_deg'],motion_limits(read(root/'config/battery-execution.json'),cfg),now=float(f['image_time']),preclearance_mm=a.stand_off_mm)
 else:
  from agent.cabinet_preview_geometry import localize
  from agent.grasp_path import plan_correction
  from vision_path_bridge import native_plan
  target=(vertical_target(m.wrist_transform(q0,h)@tcp,a.up_mm) if a.up_mm is not None
          else lateral_target(m.wrist_transform(q0,h)@tcp,a.delta_y_mm))
  path=plan_correction(m,q0,h,tcp,target,cfg['arms']['right']['joint_limits_deg'],timing_limits=motion_limits(read(root/'config/battery-execution.json'),cfg),translation_step_mm=5.,rotation_step_deg=1.)
  proposal=dict(stage='vertical_adjustment' if a.up_mm is not None else 'lateral_adjustment',native_plan=native_plan(path),window=localize(f,profile=slot,now=float(f['image_time'])),target_base_from_tool=target.tolist(),gripper_action='retain',up_mm=a.up_mm,delta_y_mm=a.delta_y_mm)

 points=np.array(proposal['native_plan']['joint_waypoints_rad'][:a.max_segments+1]);table=table_front_x(f,details=True)
 def sweep(start,end):return check_outside_sweep(m,lm,left,h,[start,end],tcp,battery,proposal['window'],table)
 sweep_results=[sweep(x,y) for x,y in zip(points,points[1:])]
 output=dict(status='planned',robot_motion=False,gripper_commands=0,source_capture=str(folder),coarse=proposal,selected_points_rad=points.tolist(),sweep=sweep_results,records=[])
 out=folder/'loaded-coarse-result.json';out.write_text(json.dumps(output,ensure_ascii=False,indent=2))
 print(json.dumps(dict(status='planned',segments=len(points)-1,capture=str(folder),image_error_before_deg=proposal.get('image_error_before_deg'),camera_roll_deg=proposal.get('camera_roll_deg'),j6_step_deg=proposal.get('j6_step_deg'),endpoint_delta_mm=((m.wrist_transform(points[-1],h)@tcp)[:3,3]-(m.wrist_transform(q0,h)@tcp)[:3,3]).__mul__(1000).tolist())),flush=True)
 if not a.execute:return
 def guard(s):
  if not s['body']['fresh'] or not all(s['arms'][v]['fresh'] for v in ('left','right')):raise ValueError('Feedback stale')
  if any(s[k] for k in ('motion_reserved','competition_reserved','chassis_active')) or s['command_epoch']!=epoch:raise ValueError('Concurrent control changed')
  if abs(s['body']['height']-h)>.03 or any(abs(s['body'][k]-v)>.02 for k,v in fixed.items()):raise ValueError('Body changed')
  if abs(s['arms']['right']['gripper']-g)>.05:raise ValueError('Clamp feedback changed')
  if max(abs(np.array(s['arms']['left']['joints'])-left))>np.radians(.5):raise ValueError('Left arm changed')
 attempted=False
 try:
  validate_start(state(),f,points,time.time())
  for i,goal in enumerate(points[1:],1):
   s=state();guard(s);before=np.array(s['arms']['right']['joints']);sweep(before,goal);guard(state())
   attempted=True;send(arm_command(goal));epoch+=1;output['robot_motion']=True
   begin=time.monotonic();history=[];target=m.wrist_transform(goal,h)@tcp
   while time.monotonic()-begin<12:
    s=state();guard(s);row=s['arms']['right'];q=np.array(row['joints']);pose=m.wrist_transform(q,h)@tcp
    # Monitor measured joints around the validated segment; retain original driver limits.
    low=np.minimum(before,goal)-np.radians(1.5);high=np.maximum(before,goal)+np.radians(1.5)
    if np.any(q<low) or np.any(q>high):raise ValueError('Measured joints left planned corridor')
    pe=float(np.linalg.norm(pose[:3,3]-target[:3,3])*1000);re=float(np.degrees(Rotation.from_matrix(pose[:3,:3].T@target[:3,:3]).magnitude()));je=float(np.max(abs(q-goal)))
    stamp=row['timestamp']
    if not history or stamp>history[-1][0]:history.append((stamp,q.copy()))
    history=[v for v in history if stamp-v[0]<.5]
    if len(history)>3 and history[-1][0]-history[0][0]>.3 and np.max(np.ptp([v[1] for v in history],axis=0))<.003 and pe<=(1. if a.up_mm is not None or a.delta_y_mm is not None or a.compensate_up_mm is not None else 5.) and re<=(.4 if a.axis_step_deg is not None or a.align_top or a.j6_step_deg is not None else 2.) and je<np.radians(1.5):break
    time.sleep(.08)
   else:raise ValueError('Measured arrival timeout')
   rec=dict(segment=i,position_error_mm=pe,rotation_error_deg=re,q=q.tolist(),delta_base_mm=((pose[:3,3]-(m.wrist_transform(q0,h)@tcp)[:3,3])*1000).tolist());output['records'].append(rec);out.write_text(json.dumps(output,ensure_ascii=False,indent=2));print(json.dumps(rec),flush=True)
  output['status']=('axis_adjustment_arrived' if a.axis_step_deg is not None else ('image_roll_step_arrived' if a.align_top or a.j6_step_deg is not None else ('vertical_adjustment_arrived' if a.up_mm is not None else ('lateral_adjustment_arrived' if a.delta_y_mm is not None else 'outside_arrived')))) if len(points)==len(proposal['native_plan']['joint_waypoints_rad']) else 'bounded_correction_arrived';output['final_state']=state()
 except Exception as exc:
  output['status']='held';output['reason']=str(exc)
  if attempted:
   try:output['stop']=send(dict(type='arm.stop',arm='right'))
   except Exception as e:output['stop_error']=str(e)
  output['final_state']=state()
  output['actual_delta_base_mm']=((m.wrist_transform(output['final_state']['arms']['right']['joints'],h)@tcp)[:3,3]-(m.wrist_transform(q0,h)@tcp)[:3,3]).__mul__(1000).tolist()
 out.write_text(json.dumps(output,ensure_ascii=False,indent=2));print(json.dumps(dict(status=output['status'],reason=output.get('reason'),report=str(out),gripper_commands=0,actual_delta_base_mm=output.get('actual_delta_base_mm',output['records'][-1]['delta_base_mm'] if output['records'] else None))),flush=True)
if __name__=='__main__':main()
