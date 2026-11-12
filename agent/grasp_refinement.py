"""Bounded correction after a settled approach and a new reviewed D405 scene.

This is a deterministic decision, not motion permission. The caller enforces the
post-action capture barrier and provides locally validated policy, never LLM values.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .grasp_geometry import rigid


def assess(current, previous, policy, *, corrections, total_mm):
    required={'max_corrections','max_target_shift_mm','max_step_mm','max_total_correction_mm',
              'position_tolerance_mm','orientation_tolerance_deg','lift_tolerance_rad'}
    if not isinstance(policy,dict) or set(policy)!=required:
        raise ValueError('Explicit local correction policy required')
    budget=policy['max_corrections']
    if (type(budget) is not int or not 0<=budget<=3 or type(corrections) is not int
            or not 0<=corrections<=budget or type(total_mm) not in (int,float)
            or not np.isfinite(total_mm) or total_mm<0):
        raise ValueError('Invalid correction budget or receipt')
    for key in ('max_target_shift_mm','max_step_mm','position_tolerance_mm'):
        a=np.asarray(policy[key],float)
        if a.shape!=(3,) or not np.isfinite(a).all() or np.any(a<=0):
            raise ValueError('Finite positive XYZ correction bounds required')
    for key in ('max_total_correction_mm','orientation_tolerance_deg','lift_tolerance_rad'):
        value=policy[key]
        if type(value) not in (int,float) or not np.isfinite(value) or value<=0:
            raise ValueError('Finite positive correction policy required')
    if policy['orientation_tolerance_deg']>=90 or total_mm>policy['max_total_correction_mm']:
        raise ValueError('Invalid correction rotation or spent budget')
    for key,value in [('arm','right'),('frame','base_link'),('unit','mm')]:
        if current.get(key)!=value or previous.get(key)!=value:raise ValueError('Correction frame mismatch')
    for key in ('head_observation_id','calibration_id','pick_setup_sha256','handeye_sha256',
                'profile_sha256','urdf_sha256','control_config_sha256'):
        if not current.get(key) or current[key]!=previous.get(key):
            raise ValueError('Correction target or calibration changed: '+key)
    for key in ('observation_id','snapshot_id'):
        if not current.get(key) or not previous.get(key) or current[key]==previous[key]:
            raise ValueError('New post-approach observation required')
    for key in ('T_wrist_from_tool','R_base_from_tool'):
        if not np.array_equal(current[key],previous[key]):raise ValueError('Correction tool frame changed')
    rigid(current['T_wrist_from_tool'])
    actual=rigid(current['actual_base_from_tool'])
    target=np.eye(4);target[:3,:3]=np.asarray(current['R_base_from_tool'],float);rigid(target)
    endpoints=[]
    for item in (previous,current):
        line=item['contact_path'];points=np.asarray([line['pre_mm'],line['end_mm']],float)
        if (line.get('axis')!='+X' or line.get('fixed_yz') is not True or points.shape!=(2,3)
                or not np.isfinite(points).all() or not 0<points[1,0]-points[0,0]<=200
                or not np.allclose(points[1,1:],points[0,1:],atol=1e-6,rtol=0)):
            raise ValueError('Invalid fixed-YZ insertion line')
        endpoints.append(points)
    before,after=endpoints
    if not np.allclose(before[1]-before[0],after[1]-after[0],atol=1e-6,rtol=0):
        raise ValueError('Pregrasp clearance changed during correction')
    lift=np.asarray([current['lift_rad'],previous['lift_rad']],float)
    if lift.shape!=(2,) or not np.isfinite(lift).all() or np.any(lift<0) or np.any(lift>18):
        raise ValueError('Invalid paired lift feedback')
    delta=after[0]-actual[:3,3]*1000
    shift=after[1]-before[1]
    angle=float(np.degrees(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude()))
    result=dict(status='reacquire_required',observation_id=current['observation_id'],
        previous_observation_id=previous['observation_id'],correction_mm=None,
        measured_pregrasp_error_mm=delta.tolist(),target_shift_mm=shift.tolist(),
        orientation_error_deg=angle,remaining_corrections=budget-corrections,
        proposed_total_correction_mm=total_mm,executable=False)
    if abs(lift[0]-lift[1])>policy['lift_tolerance_rad']:
        return result|{'reason':'lift_changed'}
    if np.any(np.abs(shift)>policy['max_target_shift_mm']):
        return result|{'reason':'target_changed_too_far'}
    if angle>policy['orientation_tolerance_deg']:
        return result|{'reason':'tool_orientation_changed'}
    if np.all(np.abs(delta)<=policy['position_tolerance_mm']):
        return result|dict(status='aligned',correction_mm=[0.,0.,0.],reason='measured_pregrasp_aligned')
    distance=float(np.linalg.norm(delta))
    if np.any(np.abs(delta)>policy['max_step_mm']):return result|{'reason':'correction_too_large'}
    if corrections>=budget or total_mm+distance>policy['max_total_correction_mm']:
        return result|{'reason':'correction_budget_exhausted'}
    return result|dict(status='correction_required',correction_mm=delta.tolist(),
        proposed_total_correction_mm=total_mm+distance,reason='new_scene_requires_bounded_correction')
