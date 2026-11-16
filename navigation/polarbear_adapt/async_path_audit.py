"""Bounded background audits using the unchanged continuous footprint checker."""
from collections import OrderedDict,Counter
from concurrent.futures import ThreadPoolExecutor
import hashlib,math,time
import numpy as np
from path_audit import AuditGrid,_convex_hull
from matplotlib.path import Path


class AuditPending(ValueError):
    def __init__(self):super().__init__('path_audit_pending')


def classes(values):
    return np.where(values==100,100,np.where(values<0,-1,0)).astype(np.int8)


class CoverageGrid(AuditGrid):
    """Conservative cell coverage; collision geometry still comes from AuditGrid."""
    def __init__(self,grid):
        super().__init__(grid.grid.copy(),grid.origin.copy(),grid.res,grid.poly.copy())
        self.coverage=np.zeros(self.grid.shape,dtype=bool)
        self.outside=False
    def _inspect_vertices(self,vertices,margin=0.):
        lo=np.floor((vertices.min(0)-margin-self.origin)/self.res).astype(int)-1
        hi=np.floor((vertices.max(0)+margin-self.origin)/self.res).astype(int)+1
        # A grid square touches the convex swept hull exactly when its centre
        # lies in the hull expanded by an axis-aligned half-cell square.
        # Certify free cells too: a future obstacle there must invalidate.
        half=self.res/2+margin
        offsets=np.array([[-half,-half],[-half,half],[half,-half],[half,half]])
        expanded=_convex_hull((np.asarray(vertices)[:,None,:]+offsets).reshape(-1,2))
        y,x=np.mgrid[lo[1]:hi[1]+1,lo[0]:hi[0]+1];x=x.ravel();y=y.ravel()
        centres=self.origin+(np.c_[x,y]+.5)*self.res
        roundoff=2e-10*(1.+np.max(np.abs(expanded)))
        touch=Path(expanded).contains_points(centres,radius=roundoff)
        inside=(x>=0)&(y>=0)&(x<self.grid.shape[1])&(y<self.grid.shape[0])
        self.outside |= bool(np.any(touch&~inside))
        self.coverage[y[touch&inside],x[touch&inside]]=True
        return super()._inspect_vertices(vertices,margin)


class Certificate:
    def __init__(self,grid,result):
        self.origin=grid.origin;self.res=grid.res;self.poly=grid.poly;self.result=result
        self.shape=grid.grid.shape;self.outside=grid.outside
        self.y,self.x=np.nonzero(grid.coverage)
        self.values=classes(grid.grid[self.y,self.x])
    def compatible(self,grid):
        if grid.res!=self.res or not np.array_equal(grid.poly,self.poly):return False
        if self.outside and (grid.grid.shape!=self.shape or not np.array_equal(grid.origin,self.origin)):return False
        offset=(self.origin-grid.origin)/self.res
        rounded=np.rint(offset).astype(int)
        if not np.allclose(offset,rounded,rtol=0,atol=1e-6):return False
        x=self.x+rounded[0];y=self.y+rounded[1]
        if np.any(x<0) or np.any(y<0) or np.any(x>=grid.grid.shape[1]) or np.any(y>=grid.grid.shape[0]):return False
        return np.array_equal(self.values,classes(grid.grid[y,x]))

    def difference(self,grid):
        """Bounded explanation only; compatible() remains the safety decision."""
        if grid.res!=self.res or not np.array_equal(grid.poly,self.poly):return dict(reason='resolution_or_footprint')
        offset=(self.origin-grid.origin)/self.res;shift=np.rint(offset).astype(int)
        if not np.allclose(offset,shift,rtol=0,atol=1e-6):return dict(reason='nonintegral_origin_shift')
        x=self.x+shift[0];y=self.y+shift[1]
        if (self.outside and (grid.grid.shape!=self.shape or not np.array_equal(grid.origin,self.origin))) or np.any(x<0) or np.any(y<0) or np.any(x>=grid.grid.shape[1]) or np.any(y>=grid.grid.shape[0]):return dict(reason='map_boundary_changed')
        values=classes(grid.grid[y,x]);changed=np.flatnonzero(self.values!=values)
        return dict(reason='covered_cell_classes_changed',changed_cells=len(changed),cells=[dict(x=float(grid.origin[0]+(x[i]+.5)*grid.res),y=float(grid.origin[1]+(y[i]+.5)*grid.res),before=int(self.values[i]),after=int(values[i])) for i in changed[:4]])


def run_audit(snapshot,route):
    return Certificate(snapshot,AuditGrid.along(snapshot,route))


class AsyncAuditor:
    """Called by one ROS thread. Worker only reads its own numpy snapshots."""
    def __init__(self):
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='path-audit')
        self.running=None;self.next=None;self.cache=OrderedDict();self.owner=None;self.closed=False
        self.counts=Counter();self.last_invalidation=None;self.worker_started=0.
    @property
    def pending_count(self):return int(self.running is not None)+int(self.next is not None)
    def diagnostic(self):
        return dict(self.counts,pending=self.pending_count,cache_entries=len(self.cache),worker_age_ms=0. if self.running is None else (time.monotonic()-self.worker_started)*1000,last_invalidation=self.last_invalidation)
    def close(self):
        self.closed=True;self.next=None;self.cache.clear()
        self.executor.shutdown(wait=False,cancel_futures=True)
    def context(self,owner):
        if owner!=self.owner:self.owner=owner;self.cache.clear();self.next=None
    def key(self,grid,route,frame,owner):
        digest=hashlib.sha256(np.asarray(route,dtype=np.float64).tobytes()).digest()
        return (owner,frame,float(grid.res),grid.poly.tobytes(),digest)
    def poll(self):
        if self.running is not None and self.running[1].done():
            key,future=self.running;self.running=None
            if key[0]==self.owner:
                self.cache[key]=future.result();self.cache.move_to_end(key)
                self.counts['completed']+=1
                while len(self.cache)>8:self.cache.popitem(last=False)
            if self.next is not None:
                key,snapshot,route=self.next;self.next=None
                if key[0]==self.owner:
                    self.running=(key,self.executor.submit(run_audit,snapshot,route));self.worker_started=time.monotonic();self.counts['submitted']+=1
    def cached(self,grid,route,frame,owner):
        self.context(owner);self.poll();key=self.key(grid,route,frame,owner)
        cert=self.cache.get(key)
        if cert is not None and cert.compatible(grid):
            self.counts['cache_hits']+=1;return cert.result
        if cert is not None:
            self.counts['invalidated']+=1;self.last_invalidation=dict(frame=frame,**cert.difference(grid))
        return None
    def check(self,grid,route,frame,owner):
        if self.closed:raise RuntimeError('path auditor closed')
        route=np.asarray(route,dtype=float)
        result=self.cached(grid,route,frame,owner)
        if result is not None:return result
        key=self.key(grid,route,frame,owner)
        if self.running is None:
            self.running=(key,self.executor.submit(run_audit,CoverageGrid(grid),route.copy()));self.worker_started=time.monotonic();self.counts['submitted']+=1
        elif self.running[0]!=key and (self.next is None or self.next[0]!=key):
            # One latest pending query, never an unbounded executor queue.
            self.next=(key,CoverageGrid(grid),route.copy())
        raise AuditPending()


class NonblockingGrid(AuditGrid):
    """Long queries use a certified worker result; short command sweeps stay live."""
    def __init__(self,grid,auditor,frame,owner):
        super().__init__(grid.grid,grid.origin,grid.res,grid.poly)
        self.auditor=auditor;self.frame=frame;self.owner=owner
    def along(self,poses):
        route=np.asarray(poses,dtype=float)
        if len(route)==0:return AuditGrid.along(self,route)
        ends=np.vstack((route[1:],route[-1:]));delta=ends-route
        yaw=np.arctan2(np.sin(delta[:,2]),np.cos(delta[:,2]))
        travel=np.linalg.norm(delta[:,:2],axis=1)+self.radius*np.abs(yaw)
        samples=np.maximum(1,np.ceil(travel/(self.res*.5))).sum()
        # Routing threshold only: no sample, margin, collision or freshness limit changes.
        if samples<=96:return AuditGrid.along(self,route)
        return self.auditor.check(self,route,self.frame,self.owner())
