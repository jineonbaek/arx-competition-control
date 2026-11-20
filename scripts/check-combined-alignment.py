"""Read-only candidate planning using one fresh paired RGB-D exposure."""
from pathlib import Path
import json,sys,urllib.request,http.cookiejar,subprocess,hashlib,time,os
import numpy as np
from scipy.spatial.transform import Rotation
R=Path('/home/arx/arx-competition-control');W=Path('/home/arx/arx-place-hold-20261116')
sys.path[:0]=[str(W/'web/src'),str(W)]
from handeye_column import ColumnModel
from agent.visual_battery_level import d435_observation,motion_limits
from agent.held_placement_geometry import effective_tcp
from agent.cabinet_preview_geometry import localize,opening
from agent.shallow_aperture import evaluate_shallow_fit
from agent.grasp_path import plan_correction
from held_insertion_sweep import check_outside_sweep
from fast_visual_level import table_front_x
load=lambda p:json.loads(p.read_text())
url='http://127.0.0.1:9983';op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()));op.open(url,timeout=5).read()
report={'status':'rejected','physical_execution':False}
try:
    state=json.load(op.open(url+'/api/state',timeout=4));q=np.array(state['arms']['right']['joints']);h=state['body']['height']
    assert state['arms']['right']['fresh'] and state['body']['fresh'] and not state['chassis_active']
    request=urllib.request.Request(url+'/api/grasp-scene',json.dumps({'camera':'d435'}).encode(),{'Origin':url,'Content-Type':'application/json'})
    cap=json.load(op.open(request,timeout=25))
    with np.load(R/'web/hardware/runtime/grasp-head/frame.npz',allow_pickle=False) as f:scene_frame={k:f[k] for k in f.files}
    if str(scene_frame['id'])!=cap['id']:raise ValueError('Scene frame changed')
    slot=opening(load(R/'config/cabinet-profiles.json'),'demo');window=localize(scene_frame,slot);table=table_front_x(scene_frame,details=True)
    subprocess.run([str(R/'.venv/bin/python'),str(W/'scripts/estimate-held-d435.py')],cwd=R,check=True,capture_output=True,timeout=30)
    pose=Path((W/'reports/d435-visual-level-active/latest-pose-path.txt').read_text())
    with np.load(pose.parent/'frame.npz',allow_pickle=False) as f:frame={k:f[k] for k in f.files}
    if str(frame['id'])!=cap['id']:raise ValueError('Frame changed')
    now=json.load(op.open(url+'/api/state',timeout=4));q=np.array(now['arms']['right']['joints']);h=now['body']['height']
    if np.max(abs(q-frame['joints']))>np.radians(.2) or abs(h-float(frame['lift_rad']))>.02:raise ValueError('Arm changed since exposure')
    m=ColumnModel(side='right');left=ColumnModel(side='left');tcp=np.array(load(R/'config/right-grip-tcp.json')['T_wrist_from_tool']);contact=np.array(load(R/'web/public/cad/battery-grip.json')['points_object_mm']['tip'])
    sha=hashlib.sha256((R/'web/cad/gripper-v2/battery-print.stl').read_bytes()).hexdigest()
    observation=d435_observation(frame,load(pose),sha);observed_tool=m.wrist_transform(frame['joints'],float(frame['lift_rad']))@tcp
    attachment=np.linalg.inv(observed_tool)@np.array(observation['T_base_from_object']);effective=effective_tcp(tcp,attachment,contact)
    config=load(R/'web/config/control.json');battery=load(R/'config/battery-envelope.json')
    T=m.wrist_transform(q,h)@effective;old=evaluate_shallow_fit(T,window,slot,battery,contact)
    L=np.array(window['left_axis_base']);U=np.array(window['up_axis_base']);axis=np.cross(L,U);axis/=np.linalg.norm(axis)
    up=float(os.environ.get('ALIGN_UP_MM','1'))
    if up not in (1.,2.,3.):raise ValueError('Invalid bounded trial')
    goal=T.copy();goal[:3,:3]=Rotation.from_rotvec(axis*np.radians(1.5)).as_matrix()@T[:3,:3];goal[:3,3]+=(-5*L+up*U)/1000
    after=evaluate_shallow_fit(goal,window,slot,battery,contact)
    report.update(frame=str(frame['id']),initial_fit=old,predicted_fit=after,candidate=dict(right_mm=5,up_mm=up,rotation_deg=1.5),observation_age_s=time.time()-float(frame['image_time']))
    path=plan_correction(m,q,h,effective,goal,config['arms']['right']['joint_limits_deg'],timing_limits=motion_limits(load(R/'config/battery-execution.json'),config),translation_step_mm=5.,rotation_step_deg=2.)
    proof=check_outside_sweep(m,left,now['arms']['left']['joints'],h,path['joint_waypoints_rad'],effective,battery|dict(contact_mm=contact.tolist()),window,table,cabinet_margin_mm=.5,_table_margin_mm=.5)
    report.update(status=('candidate_path_checked_offline' if after['qualified_fit'] else 'outside_path_checked_aperture_not_qualified'),path=path,proof=proof,window=window,table=table,effective_tcp=effective.tolist(),start_joints=q.tolist(),height=h,gripper=now['arms']['right']['gripper'])
except Exception as e:report['reason']=repr(e)
(W/'reports/combined-alignment-check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps({k:report.get(k) for k in ('status','reason','frame','candidate','physical_execution')},ensure_ascii=False),flush=True)
if 'predicted_fit' in report:print(json.dumps({'initial_gap_mm':report['initial_fit']['nominal_clearance_mm'],'candidate_gap_mm':report['predicted_fit']['nominal_clearance_mm'],'standoff_mm':report['predicted_fit']['leading_standoff_mm']}),flush=True)
