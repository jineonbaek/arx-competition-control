"""Check grid squares touched by the padded CAD footprint along a planar path."""
import math
import numpy as np
from matplotlib.path import Path


def heading(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def planar(p):
    return np.array([p.position.x, p.position.y, heading(p.orientation)])


def _convex_hull(points):
    """Monotone-chain hull of the endpoint polygons; no geometry dependency."""
    points = sorted(set(map(tuple, points)))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    low, high = [], []
    for p in points:
        while len(low) >= 2 and cross(low[-2], low[-1], p) <= 0:
            low.pop()
        low.append(p)
    for p in reversed(points):
        while len(high) >= 2 and cross(high[-2], high[-1], p) <= 0:
            high.pop()
        high.append(p)
    return np.asarray(low[:-1] + high[:-1])


class AuditGrid:
    def __init__(self, grid, origin, resolution, footprint):
        self.grid = np.asarray(grid)
        self.origin = np.asarray(origin)
        self.res = resolution
        self.poly = np.asarray(footprint)
        self.radius = np.max(np.linalg.norm(self.poly, axis=1))

    def _vertices(self, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return self.poly @ [[c,s],[-s,c]] + [x,y]

    def inspect(self, pose, margin=0.0):
        return self._inspect_vertices(self._vertices(pose), margin)

    def _inspect_vertices(self, vertices, margin=0.0):
        half = self.res / 2 + margin
        lo = np.floor((vertices.min(0) - margin - self.origin) / self.res).astype(int) - 1
        hi = np.floor((vertices.max(0) + margin - self.origin) / self.res).astype(int) + 1
        j,i = np.mgrid[lo[1]:hi[1]+1, lo[0]:hi[0]+1]
        i,j = i.ravel(),j.ravel()
        inside = (i>=0)&(j>=0)&(i<self.grid.shape[1])&(j<self.grid.shape[0])
        values=np.full(len(i),-1);values[inside]=self.grid[j[inside],i[inside]]
        relevant=(values==100)|(values<0)
        if not relevant.any():
            return dict(unknown=0,lethal=0,first_cell=None)
        centres = self.origin + (np.c_[i[relevant], j[relevant]] + .5) * self.res
        values=values[relevant]
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
        vals=values[touch]
        hard=vals==100; unknown=vals<0
        hit=hard if hard.any() else unknown
        where=centres[touch][hit]
        return dict(unknown=int(unknown.sum()),lethal=int(hard.sum()),
                    first_cell=(where[0].tolist() if len(where) else None))

    def along(self, poses):
        first=None; maximum=[0,0]; count=0
        poses = np.asarray(poses, dtype=float)
        for idx,a in enumerate(poses):
            b=poses[min(idx+1,len(poses)-1)]
            da=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
            travel=np.linalg.norm(b[:2]-a[:2])+self.radius*abs(da)
            steps=max(1,math.ceil(travel/(self.res*.5)))
            for step in range(steps):
                t=step/steps
                pose=np.r_[a[:2]+t*(b[:2]-a[:2]),a[2]+t*da]
                u=(step+1)/steps
                end=np.r_[a[:2]+u*(b[:2]-a[:2]),a[2]+u*da]
                vertices = _convex_hull(np.vstack((self._vertices(pose), self._vertices(end))))
                # For any body point, ||rotation''(t)|| <= r * dtheta**2.
                # Endpoint interpolation error is therefore <= r*dtheta**2/8.
                # Translation interpolates exactly. The endpoint hull enlarged
                # by this error disk covers the entire continuous body motion;
                # testing enlarged grid squares also encloses that disk.
                rotation_error = self.radius * (da/steps)**2 / 8.
                roundoff = 1e-10 * (1. + np.max(np.abs(vertices)))
                v=self._inspect_vertices(vertices,margin=rotation_error+roundoff)
                maximum=[max(maximum[0],v['unknown']),max(maximum[1],v['lethal'])]
                if first is None and v['first_cell'] is not None:
                    first=dict(path_index=idx,pose=pose.tolist(),**v)
                count+=1
        return dict(samples=count,max_unknown=maximum[0],max_lethal=maximum[1],first=first)
