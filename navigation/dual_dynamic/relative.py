"""Diagnostic SE(3) relative pose only; not a fused chassis state estimator."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def interpolate(ts, positions, quaternions, target, max_gap=.15):
    ts=np.asarray(ts,dtype=np.int64);p=np.asarray(positions,float);q=np.asarray(quaternions,float)
    if len(ts)<2 or np.any(np.diff(ts)<=0):raise ValueError('strictly increasing timestamps required')
    if p.shape!=(len(ts),3) or q.shape!=(len(ts),4) or not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)):raise ValueError('invalid pose data')
    if target<ts[0] or target>ts[-1]:raise ValueError('extrapolation forbidden')
    i=min(max(int(np.searchsorted(ts,target,side='right'))-1,0),len(ts)-2)
    dt=int(ts[i+1])-int(ts[i])
    if dt>max_gap*1e9:raise ValueError('interpolation gap exceeds limit')
    fraction=(int(target)-int(ts[i]))/dt
    result=np.eye(4);result[:3,3]=p[i]*(1-fraction)+p[i+1]*fraction
    result[:3,:3]=Slerp([0.,1.],Rotation.from_quat(q[i:i+2]))([fraction]).as_matrix()[0]
    return result


def world_alignment(bottom_initial,head_initial,head_in_bottom_initial):
    """T_worldBottom_worldHead; initial prior is not a fixed dynamic extrinsic."""
    return bottom_initial @ head_in_bottom_initial @ np.linalg.inv(head_initial)


def relative_pose(bottom_pose,head_pose,alignment):
    """T_bottomIMU_headIMU at one time; includes independent LIO drift/errors."""
    return np.linalg.inv(bottom_pose) @ alignment @ head_pose
