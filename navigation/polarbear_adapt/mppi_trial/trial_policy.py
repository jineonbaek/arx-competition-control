"""Outcome and speed regulation for bounded MPPI commissioning trials."""
import math

class ReadinessHold:
    """Caller must brake during a brief outage; recovery keeps the action alive."""
    def __init__(self):self.since=None
    def update(self,ready,now):
        if ready:self.since=None;return True
        if self.since is None:self.since=now
        if now-self.since>.30:raise ValueError('input outage exceeded .30 seconds')
        return False

def target_pose(start, relative):
    x,y,a=start;f,l,w=relative;c,s=math.cos(a),math.sin(a)
    return x+c*f-s*l,y+s*f+c*l,a+w

def trial_envelope(delta, target):
    x,y,w=delta;f,l,a=target;distance=math.hypot(f,l)
    if not all(math.isfinite(v) for v in (*delta,*target)):return False
    if math.hypot(x,y)>distance+.04 or abs(w)>abs(a)+.14:return False
    if distance>.01 and abs((-l*x+f*y)/distance)>.055:return False
    return True

def trial_progress(delta, target):
    x,y,w=delta;f,l,a=target;distance=math.hypot(f,l)
    return (x*f+y*l)/distance>=.012 if distance>.01 else w*math.copysign(1.,a)>=.025

def speed_factors(measured):
    x,y,w=measured
    if not all(math.isfinite(v) for v in (x,y,w)):raise ValueError('Nonfinite measured velocity')
    return min(1.,.10/max(math.hypot(x,y),1e-9)),min(1.,.18/max(abs(w),1e-9))

def verified_arrival(action,error,drift,idle):
    return (action==[4,0] and idle and math.hypot(*error[:2])<=.045 and
            abs(error[2])<=.09 and math.hypot(*drift[:2])<=.009 and abs(drift[2])<=.02)
