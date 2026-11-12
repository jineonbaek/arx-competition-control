"""Bounded correction after a settled approach and a new reviewed D405 scene.

This is a deterministic decision, not motion permission. The caller enforces the
post-action capture barrier and provides locally validated policy, never LLM values.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .grasp_geometry import rigid


def assess(current, previous, policy, *, corrections, total_mm):
    required={'max_step_mm','max_rotation_step_deg','position_tolerance_mm','orientation_tolerance_deg'}
    if not isinstance(policy,dict) or set(policy)!=required:
        raise ValueError('Explicit local correction policy required')
    if (type(corrections) is not int or corrections<0 or type(total_mm) not in (int,float)
            or not np.isfinite(total_mm) or total_mm<0):
        raise ValueError('Invalid correction receipt')
    tolerance=np.asarray(policy['position_tolerance_mm'],float)
    if tolerance.shape!=(3,) or not np.isfinite(tolerance).all() or np.any(tolerance<=0):
        raise ValueError('Finite positive XYZ correction bounds required')
    for key in ('max_step_mm','max_rotation_step_deg','orientation_tolerance_deg'):
        value=policy[key]
        if type(value) not in (int,float) or not np.isfinite(value) or value<=0:
            raise ValueError('Finite positive correction policy required')
    if policy['max_step_mm']>10 or max(policy['orientation_tolerance_deg'],policy['max_rotation_step_deg'])>=90:
        raise ValueError('Invalid correction rotation')
    for key,value in [('arm','right'),('frame','base_link'),('unit','mm')]:
        if current.get(key)!=value or previous.get(key)!=value:raise ValueError('Correction frame mismatch')
    for key in ('head_observation_id','calibration_id','pick_setup_sha256','handeye_sha256',
                'profile_sha256','urdf_sha256','control_config_sha256'):
        if not current.get(key) or current[key]!=previous.get(key):
            raise ValueError('Correction target or calibration changed: '+key)
    for key in ('observation_id','snapshot_id'):
        if not current.get(key) or not previous.get(key) or current[key]==previous[key]:
            raise ValueError('New post-approach observation required')
    if not np.array_equal(current['T_wrist_from_tool'],previous['T_wrist_from_tool']):
        raise ValueError('Correction TCP calibration changed')
    rigid(current['T_wrist_from_tool'])
    actual=rigid(current['actual_base_from_tool'])
    target=np.eye(4);target[:3,:3]=np.asarray(current['R_base_from_tool'],float);rigid(target)
    endpoints=[]
    for item in (previous,current):
        line=item['contact_path'];points=np.asarray([line['pre_mm'],line['end_mm']],float)
        direction=np.asarray(line.get('direction_base'),float)
        delta=points[1]-points[0];distance=float(np.linalg.norm(delta))
        if (line.get('kind')!='battery_approach_axis' or points.shape!=(2,3)
                or direction.shape!=(3,) or not np.isfinite(points).all()
                or not np.isfinite(direction).all() or not 0<distance<=200
                or not np.allclose(delta/distance,direction,atol=1e-6,rtol=0)):
            raise ValueError('Invalid battery-axis insertion line')
        endpoints.append(points)
    before,after=endpoints
    old_axis=before[1]-before[0];new_axis=after[1]-after[0]
    if not np.isclose(np.linalg.norm(old_axis),np.linalg.norm(new_axis),atol=1e-6,rtol=0):
        raise ValueError('Insertion clearance changed during correction')
    lift=np.asarray([current['lift_rad'],previous['lift_rad']],float)
    if lift.shape!=(2,) or not np.isfinite(lift).all() or np.any(lift<0) or np.any(lift>18):
        raise ValueError('Invalid paired lift feedback')
    delta=after[0]-actual[:3,3]*1000
    shift=after[1]-before[1]
    angle=float(np.degrees(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude()))
    result=dict(status='reacquire_required',observation_id=current['observation_id'],
        previous_observation_id=previous['observation_id'],correction_mm=None,
        measured_pregrasp_error_mm=delta.tolist(),target_shift_mm=shift.tolist(),
        orientation_error_deg=angle,
        proposed_total_correction_mm=total_mm,executable=False)
    if np.all(np.abs(delta)<=tolerance+1e-9) and angle<=policy['orientation_tolerance_deg']+1e-9:
        return result|dict(status='aligned',correction_mm=[0.,0.,0.],reason='measured_pregrasp_aligned')
    distance=float(np.linalg.norm(delta))
    step=delta*min(1.,policy['max_step_mm']/distance) if distance else delta
    rotation=Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).as_rotvec()
    rotation*=min(1.,policy['max_rotation_step_deg']/angle) if angle else 1.
    corrected=actual.copy();corrected[:3,3]+=step/1000
    corrected[:3,:3]=actual[:3,:3]@Rotation.from_rotvec(rotation).as_matrix()
    return result|dict(status='correction_required',correction_mm=step.tolist(),
        correction_rotation_deg=np.degrees(rotation).tolist(),
        correction_target_base_from_tool=corrected.tolist(),
        proposed_total_correction_mm=total_mm+float(np.linalg.norm(step)),
        reason='new_scene_requires_bounded_correction')
