"""Prove stopped motion from distinct, post-stop source poses."""
import math

class StopWindow:
    def __init__(self,after):self.after=after;self.samples=[]
    def add(self,stamp,pose):
        if stamp<self.after or (self.samples and stamp<=self.samples[-1][0]):return False
        if not all(math.isfinite(v) for v in (stamp,*pose)):
            self.samples=[];return False
        self.samples.append((stamp,tuple(pose)))
        while len(self.samples)>1 and self.samples[1][0]<=stamp-1.:self.samples.pop(0)
        if len(self.samples)<8 or stamp-self.samples[0][0]<1.:return False
        x=[p[0] for _,p in self.samples];y=[p[1] for _,p in self.samples]
        a=self.samples[0][1][2]
        yaw=[math.atan2(math.sin(p[2]-a),math.cos(p[2]-a)) for _,p in self.samples]
        return math.hypot(max(x)-min(x),max(y)-min(y))<=.009 and max(yaw)-min(yaw)<=.02
