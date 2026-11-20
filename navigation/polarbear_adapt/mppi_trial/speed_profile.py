"""Requested SDK tiers with provisional SI model, not a measured high-speed fit."""
import math
NORMAL_SDK=.6
SLOW_SDK=.225
DEFAULT_MODE='normal'
# Forward0.30SDK/~0.22m/s is measured; high/side/yaw are planning estimates.
NORMAL_XY_SPEED=.22*.8/.3
NORMAL_YAW_SPEED=.8
# Keep the existing velocity/SDK calibration and controller base limits.
# Reduce requested controller speed and final SDK caps together.
NORMAL_PERCENT=100.*NORMAL_SDK/.8
SLOW_PERCENT=100.*SLOW_SDK/.8

def cap_sdk(command,mode='normal'):
    if mode not in ('normal','slow'):raise ValueError('unknown speed mode')
    x,y,w=map(float,command)
    if not all(math.isfinite(v) for v in (x,y,w)):raise ValueError('nonfinite SDK')
    cap=NORMAL_SDK if mode=='normal' else SLOW_SDK
    scale=min(1.,cap/max(math.hypot(x,y),1e-9))
    return x*scale,y*scale,max(-cap,min(w,cap))

class SpeedMode:
    def __init__(self):self.mode=DEFAULT_MODE;self.clear_since=None
    def update(self,reason,now):
        if reason:self.mode='slow';self.clear_since=None
        elif self.mode=='slow':
            if self.clear_since is None:self.clear_since=now
            if now-self.clear_since>=.8:self.mode='normal'
        return self.mode

def scene_reason(pose,target,route,grids,command):
    """Cheap tier selection; the independent continuous collision audit still runs."""
    import numpy as np
    from continuous_track import projected_progress,wrap
    pose=np.asarray(pose);target=np.asarray(target);route=np.asarray(route)
    if np.linalg.norm(target[:2]-pose[:2])<.8:return 'near_goal'
    if abs(command[2])>.04:return 'turning'
    index,projection=projected_progress(route,pose)
    looks=[pose];distance=0.;last=projection;tangent=None
    for end in route[index+1:]:
        delta=end[:2]-last[:2];length=float(np.linalg.norm(delta))
        if length>1e-8:
            angle=math.atan2(delta[1],delta[0])
            if tangent is not None and abs(wrap(angle-tangent))>.25:return 'path_turn'
            tangent=angle
        if abs(wrap(end[2]-pose[2]))>.20:return 'heading_turn'
        distance+=length
        if distance>=.4*len(looks):looks.append(end)
        last=end
        if distance>=.9:break
    if distance<.75:return 'short_straight'
    c,s=math.cos(pose[2]),math.sin(pose[2])
    for grid,frame_pose in grids:
        if grid is None:return 'missing_grid'
        a=frame_pose[2];ca,sa=math.cos(a),math.sin(a)
        for look in looks:
            d=look[:2]-pose[:2];bx,by=c*d[0]+s*d[1],-s*d[0]+c*d[1]
            q=(frame_pose[0]+ca*bx-sa*by,frame_pose[1]+sa*bx+ca*by,a+wrap(look[2]-pose[2]))
            if grid.inspect(q,margin=.20)['first_cell'] is not None:return 'narrow_or_unknown'
    return None

def command_sweeps(pose,command,measured):
    """Expand the old low-speed footprint sweep; never shrink below measured motion."""
    from continuous_track import swept_poses
    import numpy as np
    command=np.asarray(command,dtype=float);measured=np.asarray(measured,dtype=float)
    if not np.isfinite(command).all() or not np.isfinite(measured).all():raise ValueError('nonfinite sweep')
    high=math.hypot(*command[:2])>.300001 or abs(command[2])>.300001
    xy=max(.90 if high else .36,1.25*math.hypot(*measured[:2])+.10)
    yaw=max(min(1.2,.15+1.5*abs(command[2])) if high else .60,1.5*abs(measured[2])+.05)
    return swept_poses(pose,command,measured,xy_bound=xy,yaw_bound=yaw)

def command_audit(grid,poses):
    # These bounded command sweeps must not enter the asynchronous long-route
    # certificate queue when the larger speed envelope exceeds96 samples.
    # Full geometry and grid-square interpolation are unchanged.
    from path_audit import AuditGrid
    return AuditGrid.along(grid,poses)
