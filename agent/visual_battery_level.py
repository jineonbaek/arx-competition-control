"""D435 CAD body-pose to a small wrist correction; never sends motor commands.

The visual rotation is measured by the perception backend (e.g. FoundationPose),
not reconstructed from FK. FK maps that observed error to wrist joint changes.
"""
import time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .placement_geometry import rigid


def d435_observation(frame,report,mesh_sha256):
    """Bind the existing FoundationPose report to its immutable D435 RGB-D file."""
    if (str(frame['camera_id'])!='d435' or str(frame['id'])!=report.get('id')
            or report.get('status')!='complete' or report.get('mesh_sha256')!=mesh_sha256
            or report.get('quality',{}).get('status')!='consistent'):
        raise ValueError('D435 exposure/CAD/pose result mismatch')
    stamp=float(frame['image_time'])
    if max(abs(stamp-float(frame[k])) for k in ('arm_time','body_time'))>.2:
        raise ValueError('Unpaired D435 frame')
    camera=rigid(report['T_camera_from_object']);base=rigid(report['T_base_from_object'])
    if not np.allclose(rigid(frame['base_from_color'])@camera,base,atol=1e-6,rtol=0):
        raise ValueError('D435 pose transforms disagree')
    return dict(camera='d435',id=report['id'],image_time=stamp,quality=report['quality'],
                T_base_from_object=base.tolist(),mesh_sha256=mesh_sha256)


def entry_joints_deg():
    """Same transport state as the current repository's carry-return step."""
    return [0.,0.,0.,0.,0.,-90.]


def motion_limits(grasp_policy,control):
    limits=grasp_policy['timing_limits'];driver=control['arms']['right']['trajectory_policy']
    speed=np.asarray(limits['joint_speed_deg_s'],float);acc=np.asarray(limits['joint_acceleration_deg_s2'],float)
    if (speed.shape!=(6,) or acc.shape!=(6,) or not np.isfinite(np.r_[speed,acc]).all() or min(speed.min(),acc.min())<=0):
        raise ValueError('Invalid repository timing limits')
    speed=np.minimum(speed,driver['speed_deg_s']);acc=np.minimum(acc,driver['acceleration_deg_s2'])
    return dict(joint_speed_deg_s=speed.tolist(),joint_acceleration_deg_s2=acc.tolist(),
                tcp_speed_mm_s=float(limits['tcp_speed_mm_s']),tcp_acceleration_mm_s2=float(limits['tcp_acceleration_mm_s2']))


def visual_level_step(observation,model,q,lift,tcp,limits_deg,*,now=None,tolerance_deg=1.,max_step_deg=3.,max_age_s=.8):
    now=time.time() if now is None else now
    if (observation.get('camera')!='d435' or not observation.get('id')
            or not .8<=max_age_s<=5. or not 0<=now-float(observation['image_time'])<=max_age_s
            or observation.get('quality',{}).get('status')!='consistent'):
        raise ValueError('Fresh consistent D435 battery-body pose required')
    body=rigid(observation['T_base_from_object']);q=np.asarray(q,float);tcp=rigid(tcp)
    if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('Invalid wrist feedback')
    down=body[:3,1];gravity=np.array([0.,0.,-1.])
    angle=float(np.arccos(np.clip(down@gravity,-1,1)));tilt=float(np.degrees(angle))
    if tilt>20.:raise ValueError('Visual battery tilt too large/ambiguous for a small leveling correction')
    result=dict(frame_id=observation['id'],tilt_deg=tilt,gripper_action='retain',source='d435_battery_body_pose')
    if tilt<=tolerance_deg:return result|dict(action='hold')
    axis=np.cross(down,gravity);axis/=np.linalg.norm(axis)
    desired=Rotation.from_rotvec(axis*(angle*.8)).apply(down)
    start=model.wrist_transform(q,lift)@tcp;bounds=np.radians(np.asarray(limits_deg,float))
    cap=np.radians(max_step_deg);lo=np.maximum(-cap,bounds[3:,0]-q[3:]);hi=np.minimum(cap,bounds[3:,1]-q[3:])
    def residual(delta):
        candidate=q.copy();candidate[3:]+=delta
        end=model.wrist_transform(candidate,lift)@tcp
        corrected=end[:3,:3]@start[:3,:3].T@down
        return np.r_[corrected-desired,delta*.001]
    fit=least_squares(residual,np.clip(np.zeros(3),lo,hi),bounds=(lo,hi),max_nfev=40)
    if not fit.success:raise ValueError('No bounded wrist correction found')
    target=q.copy();target[3:]+=fit.x;end=model.wrist_transform(target,lift)@tcp
    translation=float(np.linalg.norm(end[:3,3]-start[:3,3]))
    if translation>.01:
        target=q+(target-q)*(.01/translation);end=model.wrist_transform(target,lift)@tcp
    predicted=end[:3,:3]@start[:3,:3].T@down
    remaining=float(np.degrees(np.arccos(np.clip(predicted@gravity,-1,1))))
    if remaining>=tilt-.1:raise ValueError('Wrist correction does not improve observed body tilt')
    return result|dict(action='correct',joint_delta_rad=(target-q).tolist(),nominal_target_rad=target.tolist(),
        predicted_tilt_deg=remaining,tcp_shift_mm=float(np.linalg.norm(end[:3,3]-start[:3,3])*1000),
        requires_fresh_image_after_motion=True,requires_loaded_path_check=True)
