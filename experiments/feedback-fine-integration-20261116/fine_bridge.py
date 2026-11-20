"""Post-arrival D405 vertical target adapter. No hardware/ROS calls.

The output remains a proposal: its path needs current-state binding and
physical clearance validation before the existing executor may use it.
"""
import numpy as np

def fine_target(frame,q,height,arrived_at,now,T_base_from_tool,estimate):
    q=np.asarray(q,float); seen=np.asarray(frame['joints'],float)
    T=np.asarray(T_base_from_tool,float)
    stamps=np.array([float(frame[k]) for k in ('image_time','arm_time','body_time')])
    if (str(frame['camera_id'])!='d405' or q.shape!=(6,) or seen.shape!=(6,)
        or T.shape!=(4,4) or not np.isfinite(np.r_[q,seen,T.ravel(),stamps,height,arrived_at,now,float(frame['lift_rad'])]).all()
        or not 0<=now-stamps[0]<=1. or stamps[0]<=arrived_at
        or max(abs(stamps-stamps[0]))>.2
        or np.max(abs(q-seen))>np.radians(.5) or abs(height-float(frame['lift_rad']))>.05
        or not np.allclose(T[3],[0,0,0,1])
        or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)
        or np.linalg.det(T[:3,:3])<.999):
        raise ValueError('D405 frame is stale, pre-arrival, or state-mismatched')
    measurement=estimate(frame);step=float(measurement['step_mm'])
    if not np.isfinite(step) or abs(step)>3.:raise ValueError('D405 correction out of range')
    target=T.copy();target[2,3]+=step/1000.
    return dict(stage='outside_vertical_fine',source_time=float(stamps[0]),
                target_base_from_tool=target.tolist(),measurement=measurement,
                gripper_action='retain',executable=False,
                reason='Requires current path and clearance validation; no insertion included')
