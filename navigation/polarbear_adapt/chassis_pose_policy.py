"""Recovery and alignment gates for the single chassis-mask pose authority."""
import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pose_freshness import fresh

class PosePolicy:
    def __init__(self):
        self.last=None;self.alignment=None;self.count=0;self.valid=False;self.now=None
    def reset(self):
        self.last=None;self.alignment=None;self.count=0;self.valid=False;self.now=None
    def reject(self):
        self.count=0;self.valid=False
        return False
    def accept(self,stamp,now,alignment):
        reversed_clock=self.now is not None and now<self.now
        self.now=now
        if (reversed_clock or not fresh(stamp,now) or
            not all(math.isfinite(v) for v in alignment)):
            return self.reject()
        if self.last is not None and stamp<self.last:return self.reject()
        if self.alignment is not None:
            x,y,yaw=alignment;ox,oy,oa=self.alignment
            delta=math.atan2(math.sin(yaw-oa),math.cos(yaw-oa))
            if math.hypot(x-ox,y-oy)>.18 or abs(delta)>.13:self.reject()
        self.alignment=tuple(alignment)
        if stamp!=self.last:
            self.last=stamp;self.count+=1
        self.valid=self.count>=3
        return self.valid

class SnapshotLease:
    """An expired/reset snapshot cannot revive without a newer measurement."""
    def __init__(self):
        self.stamp=None;self.floor=-1;self.now=None
    def withdraw(self,floor=None):
        if self.stamp is not None:self.floor=max(self.floor,self.stamp)
        if floor is not None:self.floor=max(self.floor,int(floor))
        self.stamp=None
    def usable(self,now):
        if self.now is not None and now<self.now:self.withdraw()
        self.now=now
        if self.stamp is not None and not fresh(self.stamp,now):self.withdraw()
        return self.stamp is not None
    def offer(self,stamp,now):
        self.usable(now)
        if stamp<=self.floor or not fresh(stamp,now):self.withdraw();return False
        self.stamp=int(stamp)
        return True
