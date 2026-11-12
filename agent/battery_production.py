"""Opt-in assembly for the deterministic battery workflow; importing is inert."""
import math
from pathlib import Path

import numpy as np

from .battery_flow import plan_digest
from .battery_workflow import build_workflow
from .config import Config
from .execution_policy import load


OPERATIONS={'execute_coarse_alignment','execute_battery_approach','execute_pick'}


def review_plan(operation,plan,*,environment_clear):
    """Convert existing CAD/IK evidence plus one operator clearance into permission."""
    if operation not in OPERATIONS or not isinstance(plan,dict):
        raise ValueError('Known execution plan required')
    if operation=='execute_coarse_alignment':
        sweep=plan.get('lift',{}).get('column_sweep',{})
        observer=plan.get('observer',{})
        joints=np.asarray(observer.get('joints_rad'),float)
        ik=observer.get('status')=='observer_candidate' and joints.shape==(6,) and np.isfinite(joints).all()
        checked=(sweep.get('both_arms_column_checked') is True
                 and isinstance(observer.get('column_path'),dict))
        clear=(sweep.get('status')=='column_clear'
               and observer.get('column_path',{}).get('clear') is True)
    else:
        points=np.asarray(plan.get('joint_waypoints_rad'),float)
        timing=plan.get('timing',{})
        ik=(plan.get('status')=='kinematic_preview' and points.ndim==2
            and points.shape[1:]==(6,) and len(points)>=2 and np.isfinite(points).all())
        checked=plan.get('column_checked') is True
        clear=(checked and type(plan.get('interpolation_position_bound_mm')) in (int,float)
               and type(plan.get('sampled_orientation_error_deg')) in (int,float)
               and plan['interpolation_position_bound_mm']<=1.
               and plan['sampled_orientation_error_deg']<=.3
               and timing.get('status')=='timed_preview')
    return dict(plan_sha256=plan_digest(plan),ik_checked=bool(ik),
        collision_checked=bool(checked),collision_free=bool(clear),
        environment_collision_checked=environment_clear is True,
        environment_clear=environment_clear is True,
        method='CAD column sweep, bounded IK path and one operator clear-area confirmation')


def preflight(operation,plan,snapshot,now):
    """Keep health/rest checks; do not turn perception quality into repeated gates."""
    if (operation not in OPERATIONS or not isinstance(plan,dict) or not isinstance(snapshot,dict)
            or type(now) not in (int,float) or not math.isfinite(now)
            or snapshot.get('chassis_active') is not False
            or snapshot.get('motion_interrupted') is not False):
        return False
    review=plan.get('execution_review',{})
    if (review.get('environment_clear') is not True
            or any(plan.get(k) is not True for k in ('executable','ik_checked','collision_checked',
                                                     'collision_free','environment_collision_checked'))):
        return False
    arms=snapshot.get('arms',{});body=snapshot.get('body',{});caps=snapshot.get('capabilities',{})
    if body.get('fresh') is not True or arms.get('right',{}).get('fresh') is not True:
        return False
    if operation=='execute_coarse_alignment':
        return (arms.get('left',{}).get('fresh') is True and caps.get('lift')==''
                and caps.get('left:arm.joints')=='' and caps.get('right:arm.joints')=='')
    # right:arm.wrist includes the existing measured "lift settled" check.
    return caps.get('right:arm.wrist')=='' and caps.get('right:gripper')==''


def workflow_factory(policy_path,*,console_url,gripper_limits):
    """Load all site-owned limits before a browser can reserve motion."""
    policy=load(Path(policy_path),gripper_limits=gripper_limits)
    config=Config(console_url=console_url,motion_enabled=True)
    policies={'coarse':dict(policy['coarse'],timing_limits=policy['timing_limits']),
              'pick':policy['pick']}

    def factory(store,controller,token,environment_clear):
        reviewer=lambda operation,plan:review_plan(
            operation,plan,environment_clear=environment_clear)
        return build_workflow(config,store,controller,token,policies=policies,
            refinement_policy=policy['refinement'],timing_limits=policy['timing_limits'],
            alignment_tolerance_deg=policy['alignment_tolerance_deg'],
            j6_tolerance_deg=policy['j6_tolerance_deg'],preflight=preflight,
            review_plan=reviewer,timeout_s=policy['timeout_s'],
            clearance_mm=policy['clearance_mm'])
    return factory
