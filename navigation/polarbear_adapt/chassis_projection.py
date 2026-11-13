"""Whole unknown map cells covered by the rigid base_link projection."""
import math

import numpy as np
import shapely


class RigidProjection:
    def __init__(self, shape):
        if shape.is_empty or not shape.is_valid or shape.geom_type not in ('Polygon','MultiPolygon'):
            raise ValueError('Invalid rigid chassis projection')
        self.shape=shape
        shapely.prepare(shape)
        left,bottom,right,top=shape.bounds
        self.radius=math.hypot(max(abs(left),abs(right)),max(abs(bottom),abs(top)))

    def covered_cells(self, grid, origin, resolution, pose, unknown_only=False):
        x,y,yaw=pose
        xy=np.array([x,y]);origin=np.asarray(origin)
        lower=np.maximum(0,np.floor((xy-self.radius-origin)/resolution).astype(int))
        upper=np.minimum(grid.shape[::-1],np.floor((xy+self.radius-origin)/resolution).astype(int)+1)
        if np.any(lower>=upper):
            return np.empty(0,dtype=int)
        window=grid[lower[1]:upper[1],lower[0]:upper[0]]
        rows,cols=np.nonzero(window==-1 if unknown_only else np.ones(window.shape,bool))
        if not len(rows):
            return np.empty(0,dtype=int)
        rows+=lower[1];cols+=lower[0]
        # Test each closed cell polygon, retaining holes and concave boundaries.
        offsets=np.array([[0,0],[1,0],[1,1],[0,1],[0,0]])
        corners=(np.stack([cols,rows],axis=1)[:,None,:]+offsets)*resolution+origin-xy
        c,s=math.cos(yaw),math.sin(yaw)
        cells=shapely.polygons(corners @ np.array([[c,-s],[s,c]]))
        covered=shapely.covers(self.shape,cells)
        return rows[covered]*grid.shape[1]+cols[covered]

    def unknown_cells(self, grid, origin, resolution, pose):
        return self.covered_cells(grid,origin,resolution,pose,unknown_only=True)
