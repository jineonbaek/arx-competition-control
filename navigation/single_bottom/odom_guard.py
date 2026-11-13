"""Monotonic odometry with bounded, fresh-only stationary gap recovery."""
import math
import numpy as np

class OdomGuard:
    def __init__(self):
        self.last=None;self.anchor=None;self.pending=False;self.count=0;self.reason=''
    def begin_recovery(self,stamp,position,quaternion):
        p=np.asarray(position,float);q=np.asarray(quaternion,float)
        if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(np.r_[p,q]).all() or np.linalg.norm(q)<1e-9:
            raise ValueError('Invalid last-published anchor')
        self.last=int(stamp);self.anchor=(p.copy(),q/np.linalg.norm(q));self.pending=True;self.count=0
    def accept(self,stamp,now,position,quaternion,velocity):
        previous=self.last
        if previous is not None and stamp<=previous:
            self.reason='nonincreasing odometry timestamp';return 'fault'
        self.last=stamp
        p=np.asarray(position,float);q=np.asarray(quaternion,float);v=np.asarray(velocity,float)
        if not np.isfinite(np.r_[p,q,v]).all() or np.linalg.norm(q)<1e-9:
            self.reason='nonfinite odometry';return 'fault'
        q=q/np.linalg.norm(q)
        fresh=-100_000_000<=now-stamp<=300_000_000
        gap=previous is not None and stamp-previous>=1_000_000_000
        if self.anchor is not None and (gap or self.pending):
            old_p,old_q=self.anchor
            angle=2*math.acos(min(1.,abs(float(np.dot(old_q,q)))))
            if np.linalg.norm(p-old_p)>.02 or angle>.03 or np.linalg.norm(v[:3])>.05 or np.linalg.norm(v[3:])>.10:
                self.reason='odometry gap with motion or pose jump';return 'fault'
            if gap:
                self.pending=True;self.count=0;return 'wait'
        if self.pending:
            if not fresh or stamp-previous>250_000_000:
                self.count=0;return 'wait'
            self.count+=1
            if self.count<3:return 'wait'
            self.pending=False
        if not fresh:return 'wait'
        self.anchor=(p.copy(),q.copy())
        return 'publish'

def map_output_allowed(state,now):
    try:
        age=now-int(state['last_published_ns'])
        return state['healthy'] is True and -100_000_000<=age<=300_000_000
    except (KeyError,TypeError,ValueError):return False
