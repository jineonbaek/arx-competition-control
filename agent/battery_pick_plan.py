"""Bind reviewed visual evidence to calibrated, offline right-arm insertion IK."""
import hashlib
import json
import numpy as np
from web.src.handeye_column import ColumnModel, PACKAGE
from .grasp_geometry import ROOT, PROFILE, rigid, validate_frame
from .grasp_scene import frame_path, CALIBRATION
from .grasp_path import plan_insert
from .storage import atomic_json


def plan(registry,observation_id,call_id,setup=None):
    store=registry.store
    store.state.pop('battery_pick_plan',None)
    store.save()
    relative=store.state.get('grasp_preview')
    if not relative:raise ValueError('本轮尚无已通过复核的位姿与点云预览')
    path=(store.path/relative).resolve()
    if not path.is_relative_to(store.path):raise ValueError('预览证据路径无效')
    blob=path.read_bytes();digest=hashlib.sha256(blob).hexdigest()
    if digest!=store.state.get('grasp_preview_sha256'):raise ValueError('预览证据已变化，请重新计算')
    evidence=json.loads(blob)
    if (evidence.get('observation_id')!=observation_id or evidence.get('status')!='preview_complete'
            or evidence.get('quality',{}).get('version')!=1
            or evidence.get('quality',{}).get('status')!='consistent'):
        raise ValueError('需要当前 D405 已通过位姿与点云复核的证据')
    handoff=evidence.get('camera_handoff') or {}
    if (handoff.get('head_observation_id')!=registry._localized('d435')
            or handoff.get('wrist_observation_id')!=observation_id
            or handoff.get('quality',{}).get('version')!=1
            or handoff.get('quality',{}).get('status')!='consistent'):
        raise ValueError('缺少本轮 D435 与 D405 同目标确认')
    geometry=evidence['geometry'];line=geometry['path']
    profile_digest=hashlib.sha256(PROFILE.read_bytes()).hexdigest()
    if (geometry.get('frame')!='base_link' or geometry.get('unit')!='mm'
            or geometry.get('profile_sha256')!=profile_digest
            or line.get('axis')!='+X' or line.get('fixed_yz') is not True):
        raise ValueError('坐标、夹指参考模型或插入方向不匹配')
    observation=store.observation(observation_id)
    calibration_digest=hashlib.sha256(CALIBRATION.read_bytes()).hexdigest()
    if observation.get('calibration_sha256')!=calibration_digest:
        raise ValueError('D405 手眼外参已变化或未绑定，请重新观察')
    with np.load(frame_path(store,observation_id),allow_pickle=False) as frame:
        validate_frame(frame)
        if str(frame.get('camera_id','d405'))!='d405' or geometry['snapshot_id']!=str(frame['id']):
            raise ValueError('预览与冻结 D405 快照不一致')
        q=np.asarray(frame['joints'],float);lift=float(frame['lift_rad'])
    if q.shape!=(6,) or not np.isfinite(q).all() or not 0<=lift<=18:
        raise ValueError('冻结快照缺少有效六轴和升降反馈')
    config_blob=(ROOT/'web/config/control.json').read_bytes()
    limits=json.loads(config_blob)['arms']['right']['joint_limits_deg']
    binding=dict(observation_id=observation_id,head_observation_id=handoff['head_observation_id'],
        snapshot_id=geometry['snapshot_id'],scene_sha256=observation['scene_sha256'],
        preview_sha256=digest,profile_sha256=profile_digest,handeye_sha256=calibration_digest,
        urdf_sha256=hashlib.sha256((PACKAGE/'urdf/lift2_with_sensors.urdf').read_bytes()).hexdigest(),
        control_config_sha256=hashlib.sha256(config_blob).hexdigest(),
        arm='right',frame='base_link',unit='mm',contact_path=line,
        start_joints_rad=q.tolist(),lift_rad=lift,calibration_id=None)
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
        for key in ('approach_x_error_deg','bump_down_error_deg'):
            value=geometry['grip'][key]
            if not np.isfinite(value) or not 0<=value<=setup['alignment_tolerance_deg']:
                raise ValueError('电池朝向与水平插入或凸起朝下不兼容')
        if not isinstance(setup['timing_limits'],dict):raise ValueError('需要显式轨迹速度和加速度参数')
        result=plan_insert(ColumnModel(),q,lift,line['pre_mm'],line['end_mm'],np.eye(3),tcp,limits,
                           timing_limits=setup['timing_limits'])
        if any(abs(np.degrees(joints[5])+90)>setup['j6_tolerance_deg']
               for joints in result['joint_waypoints_rad']):
            raise ValueError('IK 夹姿不是约定 J6 -90 度，不能套用带槽夹指方向')
        binding.update(calibration_id=setup['calibration_id'],
            pick_setup_sha256=hashlib.sha256(json.dumps(setup,sort_keys=True,allow_nan=False).encode()).hexdigest(),
            T_wrist_from_tool=tcp.tolist(),R_base_from_tool=np.eye(3).tolist(),
            actual_base_from_tool=(ColumnModel().wrist_transform(q,lift)@tcp).tolist())
    result.update(binding,executable=False)
    output=store.path/'pick-plans'/call_id/'result.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    atomic_json(output,result)
    relative=str(output.relative_to(store.path));store.state['battery_pick_plan']=relative
    store.save()
    return result|{'result_file':relative}
