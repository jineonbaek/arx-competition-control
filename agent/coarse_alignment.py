"""Fixed lift plan from immutable head-camera evidence; no hardware commands."""
import hashlib
import json
from pathlib import Path
import numpy as np
from web.src.handeye_column import ColumnModel, NOMINAL_RAD_PER_M, PACKAGE
from .grasp_scene import frame_path
from .storage import atomic_json
from .lift_sweep import assess_lift
from .execution_policy import load

ROOT=Path(__file__).resolve().parents[1]

def coarse_geometry(target_mm,current_lift,model,limits=(0.,18.),*,fixed_lift_rad,
                    observer_start_z_mm,observer_rpy_deg,current_joints,limits_deg):
    target=np.asarray(target_mm,float)
    if len(limits)!=2 or not np.isfinite(limits).all() or limits[0]>=limits[1]:
        raise ValueError('升降范围无效')
    if (target.shape!=(3,) or not np.isfinite(target).all() or not np.isfinite(current_lift)
            or not np.isfinite(fixed_lift_rad)
            or not limits[0]<=current_lift<=limits[1]
            or not limits[0]<=fixed_lift_rad<=limits[1]
            or not np.isfinite(observer_start_z_mm)):
        raise ValueError('粗定位坐标或升降反馈无效')
    q=np.asarray(current_joints,float);rpy=np.asarray(observer_rpy_deg,float)
    arm_limits=np.asarray(limits_deg,float)
    if (q.shape!=(6,) or not np.isfinite(q).all() or rpy.shape!=(3,) or not np.isfinite(rpy).all()
            or arm_limits.shape!=(6,2) or not np.isfinite(arm_limits).all()):
        raise ValueError('固定观察姿态参数无效')
    zero=model.wrist_transform(np.zeros(6),0)[:3,3]*1000
    height=float(fixed_lift_rad)
    reference=zero.copy();reference[2]+=height/NOMINAL_RAD_PER_M*1000
    current_wrist=model.wrist_transform(q,float(current_lift))[:3,3]*1000
    observer_xyz=current_wrist.copy();observer_xyz[2]=float(observer_start_z_mm)
    try:
        observer_joints=np.asarray(model.inverse_wrist(q,height,observer_xyz,rpy,limits_deg),float)
        observer_path=model.check(q,observer_joints,height)
        observer=dict(kind='fixed',status='fixed_observer_candidate',
            joints_rad=observer_joints.tolist(),target_wrist_base_mm=observer_xyz.tolist(),
            rpy_deg=rpy.tolist(),column_path=observer_path,
            note='X/Y 保持当前初始腕位，Z 使用固定观察起点；移动后必须用新 D405 帧确认。')
        if not observer_path['clear']:
            observer.update(status='fixed_observer_rejected',message='固定观察姿态到中央立柱间隙不足')
    except (ValueError,TypeError) as error:
        observer=dict(kind='fixed',status='fixed_observer_unavailable',joints_rad=None,
            target_wrist_base_mm=observer_xyz.tolist(),rpy_deg=rpy.tolist(),message=str(error))
    return dict(frame='base_link',unit='mm',status='coarse_proposal',
        target_surface_base_mm=target.tolist(),reference_joints_deg=[0]*6,
        reference_wrist_base_mm=reference.tolist(),x_y_alignment='deferred_to_d405',
        lift=dict(current_rad=float(current_lift),requested_rad=height,target_rad=height,
            acceptance_rad=float(fixed_lift_rad),
            delta_rad=height-float(current_lift),limits_rad=list(limits),
            nominal_rad_per_m=NOMINAL_RAD_PER_M,fixed=True,clamped=False,
            both_arms_sweep_checked=False),
        observer=observer,tcp=None,tcp_status='pending',executable=False,ik_checked=False,
        blockers=['尚未检查升降时双臂、夹爪与环境的扫掠空间',
                  ('固定观察姿态不可达：'+observer.get('message','') if observer['status']!='fixed_observer_candidate'
                   else '固定观察姿态仅为几何候选；D405 新帧和目标可见性尚未确认'),
                  'TCP 待用户提供；不得替用旧示教偏移'],
        note='升降后先到固定观察点，再由 D405 判断目标并进入 FoundationPose 自适应抓取。')

def observer_search_geometry(observer,offset_mm,current_q,current_lift,model,limits_deg):
    if not isinstance(observer,dict) or observer.get('status')!='fixed_observer_candidate':
        raise ValueError('固定观察姿态尚未通过检查')
    if type(offset_mm) not in (int,float) or not np.isfinite(offset_mm) or abs(offset_mm)>100:
        raise ValueError('观察搜索偏移无效')
    q=np.asarray(current_q,float);target=np.asarray(observer.get('target_wrist_base_mm'),float)
    rpy=np.asarray(observer.get('rpy_deg'),float)
    if (q.shape!=(6,) or target.shape!=(3,) or rpy.shape!=(3,)
            or not np.isfinite(np.r_[q,target,rpy,current_lift]).all()):
        raise ValueError('观察搜索反馈无效')
    target=target.copy();target[2]+=float(offset_mm)
    joints=np.asarray(model.inverse_wrist(q,current_lift,target,rpy,limits_deg),float)
    path=model.check(q,joints,current_lift)
    return dict(target_wrist_base_mm=target.tolist(),joints_rad=joints.tolist(),
        column_path=path,clear=path['clear'])

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
    policies=getattr(getattr(registry,'backends',None),'policies',{})
    coarse_policy=policies.get('coarse') if isinstance(policies,dict) else None
    if not isinstance(coarse_policy,dict):
        coarse_policy=load(ROOT/'config/battery-execution.json',
            gripper_limits=config['arms']['right']['gripper_limits_raw'])['coarse']
    observer_search=coarse_policy['observer_search']
    result=coarse_geometry(location['surface_base_mm'],height,ColumnModel(),limits,
        fixed_lift_rad=coarse_policy['fixed_lift_target_rad'],
        observer_start_z_mm=observer_search['start_z_mm'],
        observer_rpy_deg=observer_search['rpy_deg'],
        current_joints=feedback['joints'],
        limits_deg=config['arms']['right']['joint_limits_deg'])
    sweep=assess_lift(feedback,result['lift']['target_rad'])
    result['lift']['acceptance_rad']=coarse_policy['lift_acceptance_rad']
    result['lift']['column_sweep']=sweep
    if sweep['status']!='column_clear':result['blockers'].append(sweep['message'])
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
