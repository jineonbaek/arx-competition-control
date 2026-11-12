"""Strict local policy input for the fixed battery executor; no defaults or I/O effects."""
import copy
import math
from pathlib import Path

import numpy as np

from .contracts import strict_json
from .grasp_path import TimedJointPath
from .motion_feedback import ArrivalMonitor


_TOP={'clearance_mm','alignment_tolerance_deg','j6_tolerance_deg','timing_limits',
      'timeout_s','coarse','pick','refinement'}
_TIMEOUTS={'execute_coarse_alignment','execute_battery_approach','execute_pick'}
_ARRIVAL={'tolerance','max_speed','settle_s','timeout_s','stall_s','max_age_s'}
_REFINEMENT={'max_corrections','max_target_shift_mm','max_step_mm',
             'max_total_correction_mm','position_tolerance_mm',
             'orientation_tolerance_deg','lift_tolerance_rad'}


def _exact(value,keys,name):
    if not isinstance(value,dict) or set(value)!=set(keys):
        raise ValueError(name+' requires exactly: '+', '.join(sorted(keys)))
    return value


def _number(value,name,*,lower=0,upper=None):
    if type(value) not in (int,float) or not math.isfinite(value) or value<=lower:
        raise ValueError(name+' must be finite and greater than '+str(lower))
    if upper is not None and value>upper:raise ValueError(name+' exceeds supported bound')
    return float(value)


def _arrival(value,size,name):
    value=_exact(value,_ARRIVAL,name)
    ArrivalMonitor(np.zeros(size),started_at=0.,**value)
    return value


def load(path,*,gripper_limits=None):
    """Load one complete site-owned policy before reserving motion control."""
    data=strict_json(Path(path).read_text(encoding='utf-8'))
    _exact(data,_TOP,'battery execution policy')
    _number(data['clearance_mm'],'clearance_mm',upper=200)
    _number(data['alignment_tolerance_deg'],'alignment_tolerance_deg',upper=45)
    _number(data['j6_tolerance_deg'],'j6_tolerance_deg',upper=45)

    timing=_exact(data['timing_limits'],
        {'joint_speed_deg_s','joint_acceleration_deg_s2','tcp_speed_mm_s','tcp_acceleration_mm_s2'},
        'timing_limits')
    TimedJointPath([np.zeros(6),np.full(6,1e-4)],1.,**timing)

    timeout=_exact(data['timeout_s'],_TIMEOUTS,'timeout_s')
    for name,value in timeout.items():_number(value,name,upper=3600)

    coarse=_exact(data['coarse'],{'reach_bound_m','fixed_lift_target_rad','lift_acceptance_rad','fixed_joint_tolerance_rad',
        'max_pair_skew_s','observer_search','lift_policy','arm_policy'},'coarse')
    _number(coarse['reach_bound_m'],'reach_bound_m')
    _number(coarse['fixed_lift_target_rad'],'coarse.fixed_lift_target_rad',upper=18)
    _number(coarse['lift_acceptance_rad'],'coarse.lift_acceptance_rad',upper=18)
    if not 0<coarse['lift_acceptance_rad']<=coarse['fixed_lift_target_rad']:
        raise ValueError('coarse.lift_acceptance_rad must not exceed fixed_lift_target_rad')
    _number(coarse['fixed_joint_tolerance_rad'],'fixed_joint_tolerance_rad')
    _number(coarse['max_pair_skew_s'],'coarse.max_pair_skew_s')
    observer=_exact(coarse['observer_search'],
        {'start_z_mm','rpy_deg','offsets_mm','z_tolerance_mm','orientation_tolerance_deg'},
        'coarse.observer_search')
    _number(observer['start_z_mm'],'coarse.observer_search.start_z_mm')
    rpy=np.asarray(observer['rpy_deg'],float)
    if rpy.shape!=(3,) or not np.isfinite(rpy).all():raise ValueError('observer_search.rpy_deg must have three values')
    offsets=np.asarray(observer['offsets_mm'],float)
    if (offsets.shape!=(8,) or not np.isfinite(offsets).all()
            or np.any(np.abs(offsets)>100) or len(set(offsets))!=len(offsets)):
        raise ValueError('observer_search.offsets_mm must be eight distinct finite values within +/-100 mm')
    _number(observer['z_tolerance_mm'],'coarse.observer_search.z_tolerance_mm',upper=20)
    _number(observer['orientation_tolerance_deg'],'coarse.observer_search.orientation_tolerance_deg',upper=45)
    _arrival(coarse['lift_policy'],1,'coarse.lift_policy')
    _arrival(coarse['arm_policy'],6,'coarse.arm_policy')

    pick=_exact(data['pick'],{'lift_tolerance_rad','max_pair_skew_s',
        'arm_policy','gripper_target','gripper_policy','hold_gripper_range'},'pick')
    _number(pick['lift_tolerance_rad'],'pick.lift_tolerance_rad')
    _number(pick['max_pair_skew_s'],'pick.max_pair_skew_s')
    _arrival(pick['arm_policy'],6,'pick.arm_policy')
    _arrival(pick['gripper_policy'],1,'pick.gripper_policy')
    target=pick['gripper_target']
    if type(target) not in (int,float) or not math.isfinite(target):
        raise ValueError('pick.gripper_target must be finite')
    if gripper_limits is not None:
        low,high=map(float,gripper_limits)
        if not low<=target<=high:raise ValueError('pick.gripper_target outside controller limits')
    hold=pick['hold_gripper_range']
    if (not isinstance(hold,list) or len(hold)!=2
            or any(type(x) not in (int,float) or not math.isfinite(x) for x in hold)
            or hold[0]>hold[1]):
        raise ValueError('pick.hold_gripper_range must be [minimum, maximum]')
    if gripper_limits is not None and not low<=hold[0]<=hold[1]<=high:
        raise ValueError('pick.hold_gripper_range outside controller limits')

    refinement=_exact(data['refinement'],_REFINEMENT,'refinement')
    budget=refinement['max_corrections']
    if type(budget) is not int or not 0<=budget<=3:
        raise ValueError('refinement.max_corrections must be an integer from 0 to 3')
    for name in ('max_target_shift_mm','max_step_mm','position_tolerance_mm'):
        values=np.asarray(refinement[name],float)
        if values.shape!=(3,) or not np.isfinite(values).all() or np.any(values<=0):
            raise ValueError('refinement.'+name+' must contain three positive finite values')
    for name in ('max_total_correction_mm','orientation_tolerance_deg','lift_tolerance_rad'):
        _number(refinement[name],'refinement.'+name,
                upper=89.999 if name=='orientation_tolerance_deg' else None)
    return copy.deepcopy(data)
