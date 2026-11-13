"""Low-speed holonomic tracking. Outputs are SDK input units. Measured safety guards use SI units."""
import math

import numpy as np


SDK_XY_MAX = 1.0
SDK_YAW_MAX = .112
SDK_XY_ACCEL = .65
SDK_YAW_ACCEL = .65 / 2.5
XY_SPEED_TARGET = .22
XY_SWEEP_SPEED = .32
YAW_SPEED_TARGET = .30
YAW_SWEEP_SPEED = .38

PERIOD = 1 / 16
POSITION_TOL = .045
YAW_TOL = .09


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def limited(vector, limit):
    return vector * min(1., limit / max(np.linalg.norm(vector), 1e-9))


class CruiseLimiter:
    """Adapt available effort gradually to load and delayed odometry."""
    def __init__(self):
        self.xy = .32
        self.yaw = SDK_YAW_MAX
        self.filtered = np.zeros(2)

    def update(self, measured, previous, dt):
        dt = float(np.clip(dt, 0., .1))
        sample = np.array([np.linalg.norm(measured[:2]), abs(measured[2])])
        self.filtered += (1. - math.exp(-dt/.32)) * (sample - self.filtered)
        for axis, target, deadband, gain, recovery, maximum in (
            (0, XY_SPEED_TARGET, .02, .9, .11, SDK_XY_MAX),
            (1, YAW_SPEED_TARGET, .03, .22, .04, SDK_YAW_MAX),
        ):
            cap = self.xy if axis == 0 else self.yaw
            effort = np.linalg.norm(previous[:2]) if axis == 0 else abs(previous[2])
            error = target - self.filtered[axis]
            if error < -deadband:
                cap -= min(2*recovery, gain*(-error-deadband))*dt
            elif error > deadband and effort >= .85*cap:
                cap += min(recovery, gain*(error-deadband))*dt
            cap = float(np.clip(cap, .015, maximum))
            if axis == 0:self.xy = cap
            else:self.yaw = cap


class OffPath(ValueError):
    pass


def projected_progress(route, pose):
    """Select a point on the polyline. Heading breaks ties at turns and crossings."""
    starts=route[:-1] if len(route)>1 else route
    ends=route[1:] if len(route)>1 else route
    delta=ends-starts
    delta[:,2]=np.arctan2(np.sin(delta[:,2]),np.cos(delta[:,2]))
    lengths=np.sum(delta[:,:2]**2,axis=1)
    fractions=np.sum((pose[:2]-starts[:,:2])*delta[:,:2],axis=1)/np.maximum(lengths,1e-12)
    pure=(lengths<1e-10)&(np.abs(delta[:,2])>1e-6)
    relative=pose[2]-starts[pure,2]
    fractions[pure]=np.arctan2(np.sin(relative),np.cos(relative))/delta[pure,2]
    points=starts+np.clip(fractions,0.,1.)[:,None]*delta
    distances=np.linalg.norm(points[:,:2]-pose[:2],axis=1)
    nearest=float(distances.min())
    if nearest>.18:raise OffPath(f'off_path distance={nearest:.3f}m')
    angles=points[:,2]-pose[2]
    score=distances+.06*np.abs(np.arctan2(np.sin(angles),np.cos(angles)))
    score[distances>nearest+.025]=np.inf
    index=int(np.argmin(score))
    return index,points[index]


def tracking_command(route, pose, target, previous, dt, stalled_for=0., limiter=None):
    """Interpolate a lookahead along the audited route, including rotation-only edges."""
    xy_error = np.linalg.norm(target[:2] - pose[:2])
    yaw_error = wrap(target[2] - pose[2])
    if xy_error <= POSITION_TOL and abs(yaw_error) <= YAW_TOL:
        return np.zeros(3), True
    index, projection = projected_progress(route, pose)
    remaining = .16
    look = projection.copy()
    # Consecutive near-coincident positions with changing heading form a pivot.
    pivot_yaw = projection[2]
    for end in route[index+1:]:
        if np.linalg.norm(end[:2]-projection[:2]) > .035:break
        pivot_yaw = end[2]
    pivot = abs(wrap(pivot_yaw-pose[2])) > .10
    for end in route[index+1:]:
        delta = end - look
        delta[2] = wrap(delta[2])
        length = np.linalg.norm(delta[:2]) + .25 * abs(delta[2])
        if length > remaining:
            look += delta * (remaining / length)
            break
        look = end.copy()
        remaining -= length
    c, s = math.cos(pose[2]), math.sin(pose[2])
    body_xy = np.array([[c, s], [-s, c]]) @ (look[:2] - pose[:2])
    yaw = wrap(look[2] - pose[2])
    if pivot:
        body_xy[:]=0.
        yaw=wrap(pivot_yaw-pose[2])
    if xy_error <= POSITION_TOL:
        body_xy[:] = 0
        yaw = yaw_error
    desired_xy = limited(body_xy * 7., min(SDK_XY_MAX, 3. * xy_error))
    desired_w = np.clip(.6 * yaw, -SDK_YAW_MAX, SDK_YAW_MAX)
    # Increase effort smoothly only while the chassis has measured no response.
    # Keep the commanded direction and retain the calibrated effort ceilings.
    boost = np.clip((stalled_for - .45) / 1.0, 0., 1.)
    xy = np.linalg.norm(desired_xy)
    if xy > .01 and boost:
        desired_xy *= 1. + boost * (min(SDK_XY_MAX, max(.16, 3.*xy_error)) / xy - 1.)
    if abs(desired_w) > .015 and boost:
        desired_w += boost * (math.copysign(SDK_YAW_MAX, desired_w) - desired_w)
    if limiter is not None:
        desired_xy = limited(desired_xy, limiter.xy)
        desired_w = np.clip(desired_w, -limiter.yaw, limiter.yaw)
    # Avoid reversing the yaw axis for minor heading noise on a straight segment.
    if xy_error > POSITION_TOL and abs(yaw) < .035:
        desired_w = 0.
    dt = np.clip(dt, 0., .1)
    cmd_xy = previous[:2] + limited(desired_xy - previous[:2], SDK_XY_ACCEL * dt)
    cmd_w = previous[2] + np.clip(desired_w - previous[2], -SDK_YAW_ACCEL * dt, SDK_YAW_ACCEL * dt)
    return np.r_[cmd_xy, cmd_w], False


def swept_poses(pose, command, measured):
    """Cover a conservative .9 s horizon for both commanded and measured motion."""
    # Inspect both the commanded envelope and the actual measured motion.
    # Model uncertainty near stiction by sweeping the full bound in command direction.
    xy_norm = np.linalg.norm(command[:2])
    commanded = np.r_[command[:2] * (XY_SWEEP_SPEED / max(xy_norm, .02)),
                      YAW_SWEEP_SPEED * np.clip(command[2] / .02, -1., 1.)]
    paths = []
    for velocity in (measured, commanded):
        p = pose.copy()
        path = [p.copy()]
        for _ in range(3):
            yaw = p[2] + .15 * velocity[2]
            c, s = math.cos(yaw), math.sin(yaw)
            p += .30 * np.array([c*velocity[0]-s*velocity[1], s*velocity[0]+c*velocity[1], velocity[2]])
            path.append(p.copy())
        paths.append(path)
    return paths
