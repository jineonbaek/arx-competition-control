"""Vision to native trajectory format. Planning only; does not publish commands."""
import numpy as np
from agent.cabinet_preview_geometry import localize,horizontal_plan
from agent.grasp_path import plan_correction
from fine_bridge import fine_target

def native_plan(path):
    points=np.asarray(path['joint_waypoints_rad'],float)
    times=np.asarray(path['timing']['time_from_start_s'],float)
    if (points.ndim!=2 or points.shape[1]!=6 or len(points)<2 or times.shape!=(len(points),)
        or not np.isfinite(points).all() or not np.isfinite(times).all()
        or times[0]!=0 or np.any(np.diff(times)<=0)):
        raise ValueError('Invalid native trajectory')
    return dict(interpolation='segment_quintic_scalar',joint_waypoints_rad=points.tolist(),time_from_start_s=times.tolist())

def coarse(frame,model,tcp,slot,battery,contact,limits,timing,*,now,preclearance_mm):
    if not 0<=now-float(frame['image_time'])<=1.:
        raise ValueError('D435 frame stale')
    if not np.isfinite(preclearance_mm) or preclearance_mm<=0:
        raise ValueError('Outside stand-off required')
    window=localize(frame,profile=slot,now=now)
    geometry=horizontal_plan(window,slot,battery,tip_point_object_mm=contact,
                             preclearance_mm=preclearance_mm,max_insertion_depth_mm=5.)
    target=np.eye(4);target[:3,:3]=geometry['R_base_from_tool']
    target[:3,3]=np.asarray(geometry['tcp_pre_base_mm'])/1000.
    path=plan_correction(model,np.asarray(frame['joints']),float(frame['lift_rad']),tcp,target,
                         limits,timing_limits=timing,translation_step_mm=5.,rotation_step_deg=2.)
    return dict(stage='outside_coarse',native_plan=native_plan(path),window=window,
                target_base_from_tool=target.tolist(),gripper_action='retain',executable=False,
                reason='Nominal TCP proposal; needs live driver binding and swept clearance validation')

def fine(frame,model,tcp,q,height,arrived_at,now,limits,timing,estimate):
    target=fine_target(frame,q,height,arrived_at,now,model.wrist_transform(q,height)@tcp,estimate)
    path=plan_correction(model,q,height,tcp,np.asarray(target['target_base_from_tool']),limits,
                         timing_limits=timing,translation_step_mm=1.,rotation_step_deg=1.)
    return target|dict(native_plan=native_plan(path))
