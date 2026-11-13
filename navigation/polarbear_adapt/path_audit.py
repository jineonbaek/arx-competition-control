"""Check grid squares touched by the padded CAD footprint along a planar path."""
import math
import numpy as np
from matplotlib.path import Path


def heading(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def planar(p):
    return np.array([p.position.x, p.position.y, heading(p.orientation)])


class AuditGrid:
    def __init__(self, grid, origin, resolution, footprint):
        self.grid = np.asarray(grid)
        self.origin = np.asarray(origin)
        self.res = resolution
        self.poly = np.asarray(footprint)
        self.radius = np.max(np.linalg.norm(self.poly, axis=1))

    def inspect(self, pose, margin=0.0):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        vertices = self.poly @ [[c,s],[-s,c]] + [x,y]
        half = self.res / 2 + margin
        lo = np.floor((vertices.min(0) - margin - self.origin) / self.res).astype(int) - 1
        hi = np.floor((vertices.max(0) + margin - self.origin) / self.res).astype(int) + 1
        j,i = np.mgrid[lo[1]:hi[1]+1, lo[0]:hi[0]+1]
        i, j = i.ravel(), j.ravel()
        inside = (i >= 0) & (j >= 0) & (i < self.grid.shape[1]) & (j < self.grid.shape[0])
        vals = np.full(len(i), -1, dtype=np.int64)
        vals[inside] = self.grid[j[inside], i[inside]]
        hazardous = (vals == 100) | (vals < 0)
        if not hazardous.any():
            return dict(unknown=0, lethal=0, first_cell=None)
        centres = self.origin + (np.column_stack((i[hazardous], j[hazardous])) + .5) * self.res
        touch = Path(vertices).contains_points(centres)
        # A boundary segment touching any part of the cell counts as overlap.
        for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
            delta = b-a
            low = np.zeros(len(centres)); high = np.ones(len(centres))
            for axis in (0,1):
                offset = centres[:,axis] - a[axis]
                if abs(delta[axis]) < 1e-12:
                    low[abs(offset)>half] = 2
                else:
                    u,v = (offset-half)/delta[axis], (offset+half)/delta[axis]
                    low = np.maximum(low,np.minimum(u,v)); high = np.minimum(high,np.maximum(u,v))
            touch |= low <= high
        vals = vals[hazardous][touch]
        hard=vals==100; unknown=vals<0
        hit=hard if hard.any() else unknown
        where=centres[touch][hit]
        return dict(unknown=int(unknown.sum()),lethal=int(hard.sum()),
                    first_cell=(where[0].tolist() if len(where) else None))

    def along(self, poses):
        first=None; maximum=[0,0]; count=0
        for idx,a in enumerate(poses):
            b=poses[min(idx+1,len(poses)-1)]
            da=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
            travel=np.linalg.norm(b[:2]-a[:2])+self.radius*abs(da)
            steps=max(1,math.ceil(travel/(self.res*.5)))
            for step in range(steps):
                t=step/steps
                pose=np.r_[a[:2]+t*(b[:2]-a[:2]),a[2]+t*da]
                # Margin bounds the motion of every footprint point until the next sample.
                v=self.inspect(pose,margin=travel/steps)
                maximum=[max(maximum[0],v['unknown']),max(maximum[1],v['lethal'])]
                if first is None and v['first_cell'] is not None:
                    first=dict(path_index=idx,pose=pose.tolist(),**v)
                count+=1
        return dict(samples=count,max_unknown=maximum[0],max_lethal=maximum[1],first=first)
