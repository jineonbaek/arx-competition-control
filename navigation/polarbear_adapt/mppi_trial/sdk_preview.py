"""Provisional MPPI velocity-to-effort estimate; not a steady SI calibration."""
import math
if __package__:
    from .effort_budget import bound
else:
    from effort_budget import bound

if __package__:
    from .speed_profile import NORMAL_XY_SPEED,NORMAL_YAW_SPEED,cap_sdk
else:
    from speed_profile import NORMAL_XY_SPEED,NORMAL_YAW_SPEED,cap_sdk
FORWARD_SPEED=REVERSE_SPEED=LATERAL_SPEED=NORMAL_XY_SPEED
YAW_SPEED=NORMAL_YAW_SPEED

def velocity_scale(vx,vy):
    if not all(math.isfinite(v) for v in (vx,vy)):raise ValueError('Nonfinite measured velocity')
    return min(1.,NORMAL_XY_SPEED/max(math.hypot(vx,vy),1e-9))

def limit_velocity(vx,vy,wz):
    if not all(math.isfinite(v) for v in (vx,vy,wz)):return 0.,0.,0.
    scale=velocity_scale(vx,vy)
    return vx*scale,vy*scale,max(-YAW_SPEED,min(wz,YAW_SPEED))

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

def extended(value,positive,negative,yaw=False):
    low,high,ml,mh=CURVES[positive if value>=0 else negative]
    if yaw:
        anchors=[(0.,0.),(ml,low*(.10/.055)),(mh,.10),(.30,.30),(.80,.80)]
    else:
        anchors=[(0.,0.),(ml,low),(mh,high),(.22,.30),(NORMAL_XY_SPEED,.80)]
    magnitude=abs(value)
    for (a,u),(b,v) in zip(anchors,anchors[1:]):
        if magnitude<=b:return math.copysign(u+(v-u)*(magnitude-a)/(b-a),value)
    return math.copysign(.80,value)

def convert(vx,vy,wz,mode='normal'):
    if not all(math.isfinite(v) for v in (vx,vy,wz)):return 0.,0.,0.
    values=(extended(vx,'forward','backward'),extended(vy,'left','right'),extended(wz,'ccw','cw',True))
    return cap_sdk(bound(values),mode)
