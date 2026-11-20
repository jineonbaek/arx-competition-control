"""Working-copy test markers and one-way measured backoff; no hardware I/O."""
import copy,math
import numpy as np

def normal_definition(value):
    d=copy.deepcopy(value)
    for p in d['points']:
        if 'test_task_id' in p:
            p['type']='task';p['point_id']=p.pop('test_task_id')
    return d

def test_definition(value,enabled):
    if type(enabled) is not bool:raise ValueError('Invalid test-mode flag')
    d=normal_definition(value)
    if enabled:
        for p in d['points']:
            if p['type']=='task':p['type']='special';p['test_task_id']=p.pop('point_id')
    return d

class Backoff:
    TARGET=.10
    MAXIMUM=.20
    SDK=.14
    def __init__(self,start):
        self.start=np.asarray(start,dtype=float);self.stopping=False;self.progress=0.
        if self.start.shape!=(3,) or not np.isfinite(self.start).all():raise ValueError('test_backoff_invalid_pose')
    def command(self,pose):
        p=np.asarray(pose,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('test_backoff_invalid_pose')
        delta=p[:2]-self.start[:2];c,s=math.cos(self.start[2]),math.sin(self.start[2])
        progress=-float(delta@np.array([c,s]));side=float(delta@np.array([-s,c]));yaw=math.atan2(math.sin(p[2]-self.start[2]),math.cos(p[2]-self.start[2]))
        self.progress=progress
        if np.linalg.norm(delta)>self.MAXIMUM or progress>self.MAXIMUM:raise ValueError('test_backoff_over_20cm')
        if progress<-.015 or abs(side)>.04 or abs(yaw)>.12:raise ValueError('test_backoff_direction_deviation')
        if progress+1e-9>=self.TARGET:self.stopping=True
        return (0.,0.,0.) if self.stopping else (-self.SDK,0.,0.)
