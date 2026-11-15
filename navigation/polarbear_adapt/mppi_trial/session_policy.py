"""Small policies shared by the interactive Omni MPPI session."""
import math
if __package__:
    from .effort_budget import load
else:
    from effort_budget import load

class CommandEpoch:
    def __init__(self):self.token=None;self.cutoff=math.inf;self.confirmed=False
    def begin(self,token,stamp):self.token=token;self.cutoff=stamp;self.confirmed=False
    def feedback(self,token,stamp):
        if token==self.token and not self.confirmed:
            self.cutoff=max(self.cutoff,stamp);self.confirmed=True
    def allows(self,source_stamp,now):
        return self.confirmed and self.cutoff<=source_stamp<=now and now-source_stamp<=.15

class OwnedGoals:
    def __init__(self):self.closed=False;self.handles=[]
    def remember(self,handle):
        if handle.accepted:
            if self.closed:handle.cancel_goal_async()
            else:self.handles.append(handle)
    def close(self):
        self.closed=True
        errors=[]
        for handle in self.handles:
            if handle.status in (1,2,3):
                try:handle.cancel_goal_async()
                except Exception as exc:errors.append(str(exc))
        return errors

def waitable_startup(reason,elapsed,accepted):
    return (not accepted and elapsed<3. and
            any(reason.startswith(prefix) for prefix in ('tf_missing','bottom_obstacles_stale',
                'dual_origin_mapping_stale','sensor_or_costmap_stale','定位 TF 过期')))

def slew_effort(previous,target,dt):
    if not valid_effort(previous) or not valid_effort(target):raise ValueError('invalid effort before slew')
    delta=[b-a for a,b in zip(previous,target)]
    step=min(.085,max(.01,dt))
    alpha=min(1.,.55*step/max(math.hypot(*delta[:2]),1e-9),.20*step/max(abs(delta[2]),1e-9))
    return tuple(a+alpha*d for a,d in zip(previous,delta))

def valid_effort(command):
    return load(command)<=1.000001

def changed_route(previous, current):
    if previous is None or not len(previous) or not len(current):return True
    a,b=previous[-1],current[-1]
    if math.hypot(a[0]-b[0],a[1]-b[1])>.02 or abs(math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2])))>.10:return True
    # Match the ordered remaining corridor, including the rectangular body's headings.
    first=0
    for p in current:
        candidates=[]
        for index in range(first,len(previous)):
            q=previous[index];distance=math.hypot(p[0]-q[0],p[1]-q[1])
            yaw=abs(math.atan2(math.sin(p[2]-q[2]),math.cos(p[2]-q[2])))
            if distance<=.08 and yaw<=.10:candidates.append((distance+.3*yaw,index))
        if not candidates:return True
        first=min(candidates)[1]
    return False
