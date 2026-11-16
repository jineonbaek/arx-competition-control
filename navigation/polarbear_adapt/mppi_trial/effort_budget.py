"""Conservative effort envelope through the attended commissioning pairs.

This limits SDK effort, not physical velocity. The existing swept-footprint
and measured-speed feedback remain required; no braking bounds are reduced.
"""
import math

XY_LIMIT=.22
YAW_LIMIT=.10
COUPLED_YAW_WEIGHT=(.22-.14)/(.10-.055)
COUPLED_LIMIT=.14+COUPLED_YAW_WEIGHT*.10

def load(command):
    x,y,w=command
    if not all(math.isfinite(v) for v in command):return math.inf
    linear=math.hypot(x,y);yaw=abs(w)
    return max(linear/XY_LIMIT,yaw/YAW_LIMIT,
               (linear+COUPLED_YAW_WEIGHT*yaw)/COUPLED_LIMIT)

def bound(command):
    scale=1./max(1.,load(command))
    return tuple(v*scale for v in command)
