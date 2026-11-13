"""Planar omniwheel kinematics and integration, independent of ROS/hardware."""
import math
import struct
import numpy as np


def decode_sdk_wheel_feedback(payload):
    if len(payload) != 8:
        raise ValueError('Expected four signed 16-bit big-endian feedback slots')
    raw = np.array(struct.unpack('>hhhh', payload), dtype=float)
    # Matches the installed official SDK, not yet a confirmation of physical units.
    return raw, raw * (10.0 / 32768.0)


def kinematic_matrix(wheels):
    rows = []
    for w in wheels:
        x, y = w['center_xy_m']
        t = np.asarray(w['rolling_direction_xy'], dtype=float)
        if not np.isfinite([x, y, *t, w['radius_m']]).all() or w['radius_m'] <= 0:
            raise ValueError('Invalid wheel geometry')
        if not np.isclose(np.linalg.norm(t), 1, atol=1e-4):
            raise ValueError('Rolling direction must be a unit vector')
        rows.append([t[0], t[1], -t[0]*y+t[1]*x])
    A = np.array(rows)
    if np.linalg.matrix_rank(A) != 3 or np.linalg.cond(A) > 100:
        raise ValueError('Wheel geometry cannot determine vx, vy, yaw rate')
    return A


def wheel_twist(sdk_values, cfg):
    wheels = cfg['wheels']
    A = kinematic_matrix(wheels)
    omega = np.array([sdk_values[w['feedback_index']]*w['feedback_sign']*cfg['rad_per_sdk_unit'] for w in wheels])
    radii = np.array([w['radius_m'] for w in wheels])
    if not np.isfinite(omega).all() or np.max(abs(omega)) > cfg['max_wheel_rad_s']:
        raise ValueError('Invalid or out-of-range wheel feedback')
    J = np.linalg.pinv(A)
    velocity = J @ (radii*omega)
    cov = J @ np.diag((radii*cfg['wheel_speed_std_rad_s'])**2) @ J.T
    return velocity, cov


def integrate(pose, velocity, dt):
    if not 0 < dt <= .25:
        raise ValueError('Integration interval must be fresh and positive')
    x, y, heading = pose
    vx, vy, wz = velocity
    a = wz*dt
    if abs(a) < 1e-8:
        dx, dy = vx*dt, vy*dt
    else:
        s, c = math.sin(a)/wz, (1-math.cos(a))/wz
        dx, dy = s*vx-c*vy, c*vx+s*vy
    cs, sn = math.cos(heading), math.sin(heading)
    return np.array([x+cs*dx-sn*dy, y+sn*dx+cs*dy, heading+a])


def integrate_with_covariance(pose, P, velocity, C, dt):
    out = integrate(pose, velocity, dt)
    F = np.eye(3)
    F[0, 2] = -(out[1]-pose[1])
    F[1, 2] = out[0]-pose[0]
    G = np.zeros((3, 3))
    for i in range(3):
        e = np.zeros(3)
        e[i] = 1e-5
        G[:, i] = (integrate(pose, velocity+e, dt)-integrate(pose, velocity-e, dt))/(2e-5)
    P = F@P@F.T+G@C@G.T
    return out, (P+P.T)*.5


def calibration_errors(cfg):
    return [key for key in ('geometry_confirmed', 'feedback_order_signs_confirmed', 'feedback_units_confirmed') if cfg.get(key) is not True]


def project_gyro_z(gyro, bias, up_axis, covariance, inflation=2.0):
    axis = np.asarray(up_axis, dtype=float)
    axis /= np.linalg.norm(axis)
    value = float(axis @ (np.asarray(gyro)-bias))
    variance = float(axis @ np.asarray(covariance) @ axis)
    return value, max(variance*inflation, 1e-5)
