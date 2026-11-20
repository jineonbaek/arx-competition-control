"""D405 body-pose leveling proposal; never controls hardware."""
import time
import numpy as np
from scipy.optimize import least_squares

def level_from_d405(observation,model,current_q,height,limits_deg,*,now=None):
    now=time.time() if now is None else float(now)
    try:
        t=float(observation["image_time"]);arm_t=float(observation["arm_time"]);body_t=float(observation["body_time"])
        recorded=np.asarray(observation["joints"],float);q=np.asarray(current_q,float);h=float(height);recorded_h=float(observation["lift_rad"])
        T=np.asarray(observation["T_base_from_object"],float);limits=np.radians(np.asarray(limits_deg,float))
        if (observation.get("camera")!="d405" or observation.get("quality")!="verified_pose" or observation.get("calibration_verified") is not True
            or q.shape!=(6,) or recorded.shape!=(6,) or T.shape!=(4,4) or limits.shape!=(6,2)
            or not np.isfinite(np.r_[q,recorded,T.ravel(),limits.ravel(),t,arm_t,body_t,h,recorded_h,now]).all()
            or not 0<=now-t<=1. or max(abs(t-arm_t),abs(t-body_t))>.2
            or np.max(np.abs(q-recorded))>np.radians(.5) or abs(h-recorded_h)>.05
            or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5)
            or np.linalg.det(T[:3,:3])<.999 or np.any(limits[:,0]>=limits[:,1])):
            raise ValueError("D405 pose or calibration unavailable, stale, or mismatched")
    except (KeyError,TypeError,IndexError) as e:raise ValueError("D405 pose metadata incomplete") from e
    down=T[:3,1];gravity=np.array([0.,0.,-1.]);angle=float(np.arccos(np.clip(down@gravity,-1,1)));tilt=float(np.degrees(angle))
    result=dict(source="d405_verified_body_pose",tilt_deg=tilt,gripper_action="retain",executable=False)
    if tilt<=1.:return result|dict(action="hold")
    axis=np.cross(down,gravity);length=np.linalg.norm(axis)
    if length<1e-9:raise ValueError("ambiguous rotation axis")
    from scipy.spatial.transform import Rotation
    desired=Rotation.from_rotvec(axis/length*(angle*.8)).apply(down)
    initial=np.asarray(model.wrist_transform(q,h),float);cap=np.radians(3.)
    lo=np.maximum(-cap,limits[3:,0]-q[3:]);hi=np.minimum(cap,limits[3:,1]-q[3:])
    if np.any(lo>=hi):raise ValueError("wrist limits leave no correction range")
    def residual(delta):
        nxt=q.copy();nxt[3:]+=delta;end=np.asarray(model.wrist_transform(nxt,h),float)
        return np.r_[end[:3,:3]@initial[:3,:3].T@down-desired,delta*.001]
    fit=least_squares(residual,np.clip(np.zeros(3),lo,hi),bounds=(lo,hi),max_nfev=40)
    if not fit.success:raise ValueError("level correction solver failed")
    nxt=q.copy();nxt[3:]+=fit.x;end=np.asarray(model.wrist_transform(nxt,h),float)
    predicted=end[:3,:3]@initial[:3,:3].T@down;remaining=float(np.degrees(np.arccos(np.clip(predicted@gravity,-1,1))))
    if remaining>=tilt-.1:raise ValueError("level correction does not improve pose")
    return result|dict(action="correct",joint_delta_deg=np.degrees(nxt-q).tolist(),nominal_target_rad=nxt.tolist(),predicted_tilt_deg=remaining)

def level_from_axis(observation,model,current_q,height,limits_deg):
    """Suggest bounded axis-leveling only; no rotation is authorized or sent."""
    try:
        axis=np.asarray(observation["axis_base"],float);q=np.asarray(current_q,float);h=float(height);limits=np.radians(np.asarray(limits_deg,float))
        if (observation.get("status")!="repeatable_axis_candidate" or axis.shape!=(3,) or q.shape!=(6,) or limits.shape!=(6,2)
            or not np.isfinite(np.r_[axis,q,h,limits.ravel()]).all() or abs(np.linalg.norm(axis)-1.)>.01
            or np.any(limits[:,0]>=limits[:,1])):raise ValueError("invalid axis observation")
    except (KeyError,TypeError,IndexError) as e:raise ValueError("axis observation incomplete") from e
    axis=axis/np.linalg.norm(axis);tilt=float(np.degrees(np.arcsin(np.clip(abs(axis[2]),0,1))))
    result=dict(source="d405_repeatable_axis_candidate",tilt_deg=tilt,gripper_action="retain",physical_accuracy_verified=False,executable=False)
    if tilt<=1.:return result|dict(action="hold")
    horizontal=axis.copy();horizontal[2]=0.;length=np.linalg.norm(horizontal)
    if length<.1:raise ValueError("axis nearly vertical; correction ambiguous")
    horizontal/=length;initial=np.asarray(model.wrist_transform(q,h),float);cap=np.radians(3.)
    lo=np.maximum(-cap,limits[3:,0]-q[3:]);hi=np.minimum(cap,limits[3:,1]-q[3:])
    if np.any(lo>=hi):raise ValueError("wrist limits leave no correction range")
    def residual(delta):
        nxt=q.copy();nxt[3:]+=delta;end=np.asarray(model.wrist_transform(nxt,h),float)
        predicted=end[:3,:3]@initial[:3,:3].T@axis
        return np.r_[predicted-horizontal,delta*.002]
    fit=least_squares(residual,np.clip(np.zeros(3),lo,hi),bounds=(lo,hi),max_nfev=40)
    if not fit.success:raise ValueError("axis leveling solver failed")
    nxt=q.copy();nxt[3:]+=fit.x;end=np.asarray(model.wrist_transform(nxt,h),float)
    displacement=np.linalg.norm(end[:3,3]-initial[:3,3])
    if displacement>.01:
        nxt=q+(nxt-q)*(.01/displacement);end=np.asarray(model.wrist_transform(nxt,h),float)
    predicted=end[:3,:3]@initial[:3,:3].T@axis
    remaining=float(np.degrees(np.arcsin(np.clip(abs(predicted[2]),0,1))))
    if remaining>=tilt-.1:raise ValueError("axis correction does not improve tilt")
    return result|dict(action="correct",joint_delta_deg=np.degrees(nxt-q).tolist(),nominal_target_rad=nxt.tolist(),predicted_tilt_deg=remaining,wrist_shift_mm=float(np.linalg.norm(end[:3,3]-initial[:3,3])*1000))
