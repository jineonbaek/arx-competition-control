"""Nominal lift proposal from immutable head-camera evidence; no hardware commands."""
import hashlib
import json
from pathlib import Path
import numpy as np
from web.src.handeye_column import ColumnModel, NOMINAL_RAD_PER_M, PACKAGE
from .grasp_scene import frame_path, CALIBRATION
from .storage import atomic_json
from .lift_sweep import assess_lift
from .observer_pose import assess_observer

ROOT=Path(__file__).resolve().parents[1]

def coarse_geometry(target_mm,current_lift,model,limits=(0.,18.)):
    target=np.asarray(target_mm,float)
    if len(limits)!=2 or not np.isfinite(limits).all() or limits[0]>=limits[1]:
        raise ValueError('升降范围无效')
    if (target.shape!=(3,) or not np.isfinite(target).all() or not np.isfinite(current_lift)
            or not limits[0]<=current_lift<=limits[1]):
        raise ValueError('粗定位坐标或升降反馈无效')
    # Use CAD neutral wrist height, never round current measured joints into an invented zero pose.
    zero=model.wrist_transform(np.zeros(6),0)[:3,3]*1000
    desired=target[2]-100.
    nominal=(desired-zero[2])*NOMINAL_RAD_PER_M/1000
    height=float(np.clip(nominal,*limits))
    reference=zero.copy();reference[2]+=height/NOMINAL_RAD_PER_M*1000
    return dict(frame='base_link',unit='mm',status='coarse_proposal',
        target_surface_base_mm=target.tolist(),reference_joints_deg=[0]*6,
        reference_wrist_base_mm=reference.tolist(),preferred_reach_up_mm=100.,
        lift=dict(current_rad=float(current_lift),requested_rad=float(nominal),target_rad=height,
            delta_rad=height-float(current_lift),limits_rad=list(limits),
            nominal_rad_per_m=NOMINAL_RAD_PER_M,clamped=bool(height!=nominal),
            preference_residual_mm=float(reference[2]-desired),both_arms_sweep_checked=False),
        lateral_difference_mm=float(target[1]-reference[1]),
        tcp=None,tcp_status='pending',executable=False,ik_checked=False,
        blockers=['D435 仅为可见表面粗定位，尚不是提手点',
                  '升降比例和 D435 CAD 外参尚非实测精度保证',
                  '尚未检查升降时双臂、夹爪与环境的扫掠空间',
                  '尚未求解可达且能看见电池的 D405 观察姿态',
                  'TCP 待用户提供；不得替用旧示教偏移'],
        note='零关节腕位仅用于按 CAD 换算升降参考，不要求机械臂回零，也不代表运动许可。')

def plan(registry,observation_id,call_id):
    store=registry.store
    relative=store.state.get('battery_observations',{}).get(observation_id)
    if not relative:raise ValueError('需要本任务已完成的 D435 观察定位')
    path=(store.path/relative).resolve()
    if not path.is_relative_to(store.path.resolve()):raise ValueError('定位证据路径无效')
    evidence=json.loads(path.read_text())
    if (evidence.get('observation_id')!=observation_id or evidence.get('camera')!='d435'
            or evidence.get('status')!='localized'):
        raise ValueError('需要已定位成功的 D435 证据')
    location=evidence['location']
    if location.get('kind')!='visible_surface_not_grip':raise ValueError('未知粗定位类型')
    with np.load(frame_path(store,observation_id),allow_pickle=False) as frame:
        if str(frame['camera_id'])!='d435':raise ValueError('非 D435 原始深度证据')
        height=float(frame['lift_rad'])
        feedback={k:frame[k].copy() for k in ('joints','left_joints','image_time','arm_time',
            'left_arm_time','body_time','lift_rad') if k in frame}
    config_path=ROOT/'web/config/control.json'
    config_blob=config_path.read_bytes()
    config=json.loads(config_blob)
    limits=config['body']['lift_limits_raw']
    result=coarse_geometry(location['surface_base_mm'],height,ColumnModel(),limits)
    sweep=assess_lift(feedback,result['lift']['target_rad'])
    result['lift']['column_sweep']=sweep
    if sweep['status']!='column_clear':result['blockers'].append(sweep['message'])
    calibration_blob=CALIBRATION.read_bytes()
    observer=assess_observer(feedback,location,result['lift']['target_rad'],
        json.loads(calibration_blob),config['arms']['right']['joint_limits_deg'])
    observer['calibration_sha256']=hashlib.sha256(calibration_blob).hexdigest()
    result['observer']=observer
    result['blockers'].remove('尚未求解可达且能看见电池的 D405 观察姿态')
    result['blockers'].append(observer['message'] if observer['status']!='observer_candidate'
        else '观察姿态仅预测视野与立柱间隙；环境、自碰撞、遮挡及移动后的同目标确认尚未验证')
    result.update(initial_joints_rad={side:(feedback[key].tolist() if key in feedback else None)
        for side,key in (('left','left_joints'),('right','joints'))},
        observation_id=observation_id,
        calibration_sha256=store.observation(observation_id).get('calibration_sha256'),
        urdf_sha256=hashlib.sha256((PACKAGE/'urdf/lift2_with_sensors.urdf').read_bytes()).hexdigest(),
        control_config_sha256=hashlib.sha256(config_blob).hexdigest())
    output=store.path/'coarse-plans'/call_id/'result.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    atomic_json(output,result)
    relative=str(output.relative_to(store.path))
    store.state['coarse_alignment']=relative
    store.save()
    return result|{'result_file':relative}
