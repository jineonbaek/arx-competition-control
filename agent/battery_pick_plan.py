"""Bind reviewed visual evidence to calibrated, offline right-arm insertion IK."""
import hashlib
import json
from pathlib import Path
import numpy as np
from web.src.handeye_column import ColumnModel, PACKAGE
from .grasp_geometry import ROOT, PROFILE, rigid, validate_frame
from .grasp_scene import frame_path, CALIBRATION, D435_CALIBRATION
from .grasp_path import plan_insert, plan_correction
from .fast_pick import fast_requested
from .storage import atomic_json


def planning_model(registry):
    """Reuse immutable CAD geometry, never a solution or a measured pose."""
    model=getattr(registry,'_pick_model',None)
    if model is None:
        controller=getattr(getattr(registry,'backends',None),'controller',None)
        model=getattr(controller,'models',{}).get('right')
    if (model is None or not getattr(model,'source_hashes',None)
            or any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h
                   for p,h in model.source_hashes.items())):
        model=ColumnModel()
    registry._pick_model=model
    return model


def materialize(registry,result,setup,scope):
    model=planning_model(registry)
    limits=json.loads((ROOT/'web/config/control.json').read_text())['arms']['right']['joint_limits_deg']
    if scope=='correction':
        motion=plan_correction(model,result['start_joints_rad'],result['lift_rad'],
            result['T_wrist_from_tool'],result['refinement']['correction_target_base_from_tool'],
            limits,timing_limits=setup['timing_limits'])
        result['correction']=motion
        result.update(motion,status='kinematic_preview')
        joints=motion['joint_waypoints_rad'][1:]
    elif scope in ('approach','insertion'):
        line=result['contact_path']
        motion=plan_insert(model,result['start_joints_rad'],result['lift_rad'],line['pre_mm'],
            line['end_mm'],result['R_base_from_tool'],result['T_wrist_from_tool'],limits,
            timing_limits=setup['timing_limits'],approach_only=scope=='approach')
        result.update(motion)
        joints=motion['joint_waypoints_rad'][1:] if scope=='approach' else motion['joint_waypoints_rad']
    else:raise ValueError('Unknown pick motion scope')
    if any(abs(np.degrees(q[5])+90)>setup['j6_tolerance_deg'] for q in joints):
        raise ValueError('IK 夹姿不是约定 J6 -90 度，不能套用带槽夹指方向')
    result['motion_scope']=scope
    return result


def plan(registry,observation_id,call_id,setup=None,*,cached=None,defer_motion=False):
    store=registry.store
    store.state.pop('battery_pick_plan',None)
    store.state.pop('battery_pick_plan_sha256',None)
    store.save()
    relative=cached['preview_file'] if cached else store.state.get('grasp_preview')
    if not relative:raise ValueError('本轮尚无已通过复核的位姿与点云预览')
    path=(store.path/relative).resolve()
    if not path.is_relative_to(store.path):raise ValueError('预览证据路径无效')
    blob=path.read_bytes();digest=hashlib.sha256(blob).hexdigest()
    expected=cached['plan']['preview_sha256'] if cached else store.state.get('grasp_preview_sha256')
    if digest!=expected:raise ValueError('预览证据已变化，请重新计算')
    evidence=json.loads(blob)
    observation=store.observation(observation_id)
    camera=observation.get('camera_id','d405')
    if (evidence.get('observation_id')!=observation_id or evidence.get('status')!='preview_complete'
            or evidence.get('quality',{}).get('version')!=1
            or evidence.get('quality',{}).get('status')!='consistent'):
        raise ValueError('需要当前 RGB-D 观察已通过位姿与点云复核的证据')
    if fast_requested(store):
        support = evidence.get('handle_support') or {}
        if (camera != 'd405' or support.get('confirmed') is not True
                or support.get('snapshot_id') != evidence['geometry']['snapshot_id']):
            raise ValueError('快流程缺少同一帧 D405 的把手深度证据')
    handoff=evidence.get('camera_handoff') or {}
    plan_f=getattr(registry,'vision_mode','d405')=='plan_f'
    if camera=='d405' and not plan_f and (handoff.get('head_observation_id')!=registry._localized('d435')
            or handoff.get('wrist_observation_id')!=observation_id
            or handoff.get('quality',{}).get('version')!=1):
        raise ValueError('缺少本轮 D435 与 D405 同目标交接证据')
    if plan_f:handoff={'head_observation_id':store.state['battery_flow']['head']}
    if camera=='d435':handoff={'head_observation_id':observation_id}
    geometry=evidence['geometry'];line=geometry['path']
    profile_digest=hashlib.sha256(PROFILE.read_bytes()).hexdigest()
    direction=np.asarray(line.get('direction_base'),float)
    if (geometry.get('frame')!='base_link' or geometry.get('unit')!='mm'
            or geometry.get('profile_sha256')!=profile_digest
            or line.get('kind')!='battery_approach_axis' or direction.shape!=(3,)
            or not np.isfinite(direction).all() or not np.isclose(np.linalg.norm(direction),1)):
        raise ValueError('坐标、夹指参考模型或插入方向不匹配')
    calibration=CALIBRATION if camera=='d405' else D435_CALIBRATION
    calibration_digest=hashlib.sha256(calibration.read_bytes()).hexdigest()
    if observation.get('calibration_sha256')!=calibration_digest:
        raise ValueError(camera.upper()+' 外参已变化或未绑定，请重新观察')
    with np.load(frame_path(store,observation_id),allow_pickle=False) as frame:
        validate_frame(frame)
        if str(frame.get('camera_id','d405'))!=camera or geometry['snapshot_id']!=str(frame['id']):
            raise ValueError('预览与冻结 '+camera.upper()+' 快照不一致')
        q=np.asarray(frame['joints'],float);lift=float(frame['lift_rad'])
    if camera=='d435' or cached:
        snapshot=registry.backends.controller.snapshot()
        q=np.asarray(snapshot['arms']['right']['joints'],float)
        lift=float(snapshot['body']['height'])
    if q.shape!=(6,) or not np.isfinite(q).all() or not 0<=lift<=18:
        raise ValueError('冻结快照缺少有效六轴和升降反馈')
    config_blob=(ROOT/'web/config/control.json').read_bytes()
    limits=json.loads(config_blob)['arms']['right']['joint_limits_deg']
    binding=dict(observation_id=observation_id,head_observation_id=handoff['head_observation_id'],
        snapshot_id=geometry['snapshot_id'],scene_sha256=observation['scene_sha256'],
        preview_sha256=digest,profile_sha256=profile_digest,handeye_sha256=calibration_digest,
        urdf_sha256=hashlib.sha256((PACKAGE/'urdf/lift2_with_sensors.urdf').read_bytes()).hexdigest(),
        control_config_sha256=hashlib.sha256(config_blob).hexdigest(),
        arm='right',frame='base_link',unit='mm',vision_mode=camera,contact_path=line,
        start_joints_rad=q.tolist(),lift_rad=lift,calibration_id=None)
    if plan_f and store.state.get('grasp_recovery'):
        binding['arrival_mode']='reference_settled'
    if setup is None:
        result=plan_insert(None,None,None,None,None,None,None,None)
    else:
        # Tool +X is insertion, +Z points away from the downward bump; never infer rotation from a pivot-only TCP.
        required={'calibration_id','parent_frame','tool_frame','translation_unit','T_wrist_from_tool',
                  'alignment_tolerance_deg','j6_tolerance_deg','timing_limits'}
        if (set(setup)!=required or setup['parent_frame']!='right_link26'
                or setup['tool_frame']!='grip_reference' or setup['translation_unit']!='m'
                or not isinstance(setup['calibration_id'],str) or not setup['calibration_id'].strip()):
            raise ValueError('需要已核验的 right_link26 到 grip_reference 米制完整 TCP')
        tcp=rigid(setup['T_wrist_from_tool'])
        for key in ('alignment_tolerance_deg','j6_tolerance_deg'):
            value=setup[key]
            if type(value) not in (int,float) or not np.isfinite(value) or not 0<value<90:
                raise ValueError('需要显式、有限的夹姿验收容差')
        value=geometry['grip']['bump_down_error_deg']
        if not np.isfinite(value) or not 0<=value<=setup['alignment_tolerance_deg']:
            raise ValueError('电池凸起朝向与约定夹姿不兼容')
        if not isinstance(setup['timing_limits'],dict):raise ValueError('需要显式轨迹速度和加速度参数')
        tool_rotation=np.asarray(geometry['grip']['R_base_from_tool'],float)
        check=np.eye(4);check[:3,:3]=tool_rotation;rigid(check)
        model=planning_model(registry)
        result=dict(status='geometry_preview',motion_scope='geometry',joint_waypoints_rad=None,timing=None)
        binding.update(calibration_id=setup['calibration_id'],
            pick_setup_sha256=hashlib.sha256(json.dumps(setup,sort_keys=True,allow_nan=False).encode()).hexdigest(),
            T_wrist_from_tool=tcp.tolist(),R_base_from_tool=tool_rotation.tolist(),
            actual_base_from_tool=(model.wrist_transform(q,lift)@tcp).tolist())
    result.update(binding,executable=False)
    if setup is not None and not defer_motion:
        materialize(registry,result,setup,'insertion')
        if not plan_f:result.pop('motion_scope',None) # Legacy combined approach/insertion proposal.
    if cached:
        # Keep the original target/evidence binding; only the measured start and trajectory change.
        for key in ('observation_id','snapshot_id','scene_sha256','preview_sha256','profile_sha256',
                    'handeye_sha256','urdf_sha256','control_config_sha256','pick_setup_sha256'):
            if result.get(key)!=cached['plan'].get(key):
                raise ValueError('缓存目标证据或标定已变化：'+key)
    if camera=='d405' and not plan_f and handoff.get('quality',{}).get('status')!='consistent':
        result['handoff_advisory']='D435/D405 跨相机质量仅为警告；执行仍依赖 D405 位姿和路径检查'
    output=store.path/'pick-plans'/call_id/'result.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    relative=str(output.relative_to(store.path))
    if not defer_motion or setup is None:
        atomic_json(output,result)
        store.state.update(battery_pick_plan=relative,
            battery_pick_plan_sha256=hashlib.sha256(output.read_bytes()).hexdigest())
        store.save()
    return result|{'result_file':relative}
