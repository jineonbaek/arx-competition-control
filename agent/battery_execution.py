"""Local assembly of existing steps. No defaults, device setup or live registration."""
import copy
import hashlib
import json
import numpy as np
from .coarse_execution import CoarseAlignmentStep
from .grasp_geometry import rigid
from .grasp_path import TimedJointPath
from .motion_feedback import PickMonitor
from .pick_execution import ApproachTrajectoryStep, StationaryApproachStep, PickTrajectoryStep

OPERATIONS={'execute_coarse_alignment','execute_battery_approach','execute_pick'}


def _trajectory(points,reach,limits):
    path=TimedJointPath(points,reach,**limits)
    return dict(interpolation='segment_quintic_scalar',time_from_start_s=path.times.tolist(),
                joint_waypoints_rad=path.q.tolist())


def prepare_step(operation,plan,controller,token,*,policies,pick_setup,preflight):
    """Bind a locally verified plan to Controller; construction sends no command.

    preflight(operation, frozen_plan, snapshot, now) must check the evidence version,
    target, swept geometry and current surroundings. The caller owns the motion token
    through the entire task and handles verified-stop cleanup. No model-supplied policy.
    """
    if operation not in OPERATIONS or not callable(preflight) or not controller.motion_valid(token):
        raise ValueError('Known operation, reserved controller and local preflight required')
    plan=copy.deepcopy(plan);policies=copy.deepcopy(policies);setup=copy.deepcopy(pick_setup)
    if plan.get('arm')!='right' or any(plan.get(k) is not True for k in
            ('executable','ik_checked','collision_checked','collision_free','environment_collision_checked')):
        raise ValueError('Complete verified path and environment checks required')
    def guard(snapshot,now):
        return preflight(operation,copy.deepcopy(plan),snapshot,now) is True
    if not guard(controller.snapshot(),controller.clock()):raise ValueError('Local preflight rejected plan')
    if operation=='execute_coarse_alignment':
        initial=plan['initial_joints_rad'];observer=plan['observer'];p=policies['coarse']
        q0=np.asarray(initial['right'],float);q1=np.asarray(observer['joints_rad'],float)
        if observer.get('status')!='observer_candidate':raise ValueError('Observer pose required')
        if np.array_equal(q0,q1):
            trajectory=dict(version=1,kind='stationary',joint_position_rad=q0.tolist())
        else:
            trajectory=dict(version=1,**_trajectory([q0,q1],p['reach_bound_m'],p['timing_limits']))
        return CoarseAlignmentStep(controller,token,lift_target_rad=plan['lift']['target_rad'],
            observer_plan=trajectory,initial_joints=initial,initial_lift_rad=plan['lift']['current_rad'],
            **{k:p[k] for k in ('fixed_joint_tolerance_rad','lift_policy','arm_policy','max_pair_skew_s')},
            lift_preflight=guard,arm_preflight=guard)
    if setup is None:raise ValueError('TCP pending')
    digest=hashlib.sha256(json.dumps(setup,sort_keys=True,allow_nan=False).encode()).hexdigest()
    if plan.get('pick_setup_sha256')!=digest or plan.get('calibration_id')!=setup['calibration_id']:
        raise ValueError('TCP or local limits changed')
    tcp=rigid(setup['T_wrist_from_tool'])
    if not np.array_equal(tcp,plan['T_wrist_from_tool']):raise ValueError('Plan TCP changed')
    model=controller.models['right'];reach=model.reach+np.linalg.norm(tcp[:3,3])
    approach=operation=='execute_battery_approach'
    if not approach and plan.get('refinement',{}).get('status')!='aligned':
        raise ValueError('Post-approach fresh observation and alignment required')
    points=np.asarray(plan['joint_waypoints_rad'],float)
    if points.ndim!=2 or points.shape[1:]!=(6,) or len(points)<2 or not np.isfinite(points).all():
        raise ValueError('Finite six-axis path required')
    stationary=False
    if approach:
        a=plan['approach'];start=np.asarray(plan['start_joints_rad'],float)
        if not np.array_equal(start,a['start_rad']) or not np.array_equal(points[0],a['end_rad']):
            raise ValueError('Approach is not bound to the checked insertion path')
        points=np.asarray([start,points[0]])
        stationary=np.array_equal(*points)
    target=np.eye(4);target[:3,:3]=np.asarray(plan['R_base_from_tool'],float)
    target[:3,3]=np.asarray(plan['contact_path']['pre_mm' if approach else 'end_mm'],float)/1000
    p=policies['pick']
    low,high=controller.config['arms']['right']['gripper_limits_raw']
    if not low<=p['gripper_target']<=high:raise ValueError('Gripper target outside configured limits')
    monitor=PickMonitor(model,tcp,target,expected_lift_rad=plan['lift_rad'],
                        started_at=controller.clock(),**p)
    if stationary:
        return StationaryApproachStep(controller,token,monitor,preflight=guard)
    trajectory=_trajectory(points,reach,setup['timing_limits'])
    cls=ApproachTrajectoryStep if approach else PickTrajectoryStep
    return cls(controller,token,monitor,trajectory,preflight=guard)
