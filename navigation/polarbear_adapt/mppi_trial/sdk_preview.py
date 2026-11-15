"""Provisional MPPI velocity-to-effort estimate; not a steady SI calibration."""
import math
if __package__:
    from .effort_budget import bound
else:
    from effort_budget import bound

# Low/high pulse inputs and transient peak response in m/s or rad/s.
CURVES = {
    'forward': (.14, .22, .042, .113),
    'backward': (.14, .22, .057, .113),
    'left': (.14, .22, .063, .106),
    'right': (.14, .22, .070, .124),
    'ccw': (.035, .055, .028, .039),
    'cw': (.035, .055, .024, .041),
}

def estimate(value, positive, negative):
    low, high, measured_low, measured_high = CURVES[positive if value >= 0 else negative]
    magnitude = abs(value)
    if magnitude <= measured_low:
        return value * low / measured_low
    fraction = min(1., (magnitude-measured_low)/(measured_high-measured_low))
    return math.copysign(low + fraction*(high-low), value)

def convert(vx, vy, wz):
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        return 0., 0., 0.
    x = estimate(vx, 'forward', 'backward')
    y = estimate(vy, 'left', 'right')
    # Current attended tests: .055 cannot reliably start yaw; .10 can.
    # Retain the reference curve shape, increase yaw effort gain only.
    # This is a commissioning candidate, not a new measured velocity curve.
    yaw = estimate(wz, 'ccw', 'cw') * (.10/.055)
    return bound((x,y,yaw))
