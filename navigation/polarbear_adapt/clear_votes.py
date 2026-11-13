"""Frame windows for the reference map and effective per-voxel sensor observations."""
from collections import Counter, deque
import numpy as np
from pathlib import Path
from shared_voxels import measured_crossings

SIZE = np.array([.05,.05,.025])


class FrameVotes:
    def __init__(self):
        self.frames = deque()
        self.counts = Counter()
        self.last = -1
        self.consecutive = deque(maxlen=3)

    def add(self, ns, crossed, consecutive=False):
        if ns <= self.last:
            return set()
        if self.last>=0 and ns-self.last>650_000_000:
            self.consecutive.clear()
        self.last = ns
        while self.frames and ns-self.frames[0][0] >= 1_000_000_000:
            _, expired = self.frames.popleft()
            for key in expired:
                self.counts[key] -= 1
                if not self.counts[key]:
                    del self.counts[key]
        keys = set(crossed)
        self.frames.append((ns, keys))
        self.counts.update(keys)
        n = len(self.frames)
        # Evaluate on misses too: the oldest sample may just have expired.
        clear = {key for key,count in self.counts.items() if 5*count >= 4*n} if n>=5 else set()
        if consecutive:
            self.consecutive.append(keys)
            if len(self.consecutive)==3:
                clear.update(set.intersection(*self.consecutive))
        return clear

    def forget(self, keys):
        for _, frame in self.frames:
            frame.difference_update(keys)
        for frame in self.consecutive:
            frame.difference_update(keys)
        for key in keys:
            self.counts.pop(key,None)


class EffectiveVotes:
    """Independent sensor streaks plus a one-second effective-observation ratio."""
    def __init__(self):
        self.last = -1
        self.frames = deque()
        self.observations = {}
        self.streaks = {}

    def add(self, ns, crossed=(), endpoints=()):
        if ns <= self.last:
            return set()
        self.last = ns
        while self.frames and ns - self.frames[0][0] >= 1_000_000_000:
            stamp, touched = self.frames.popleft()
            for key in touched:
                history = self.observations.get(key)
                if history and history[0][0] == stamp:
                    history.popleft()
                    if not history:
                        del self.observations[key]
        touched = set(crossed) | set(endpoints)
        if touched:
            self.frames.append((ns, touched))
        clear = set()
        for key in touched:
            history = self.observations.setdefault(key, deque())
            through = key in crossed and key not in endpoints
            history.append((ns, through))
            if through:
                self.streaks[key] = min(4, self.streaks.get(key, 0) + 1)
            else:
                self.streaks.pop(key, None)
            if self.streaks.get(key, 0) == 4:
                clear.add(key)
            elif len(history) >= 5 and 5 * sum(v for _, v in history) >= 4 * len(history):
                clear.add(key)
        return clear

    def forget(self, keys):
        for key in keys:
            self.observations.pop(key, None)
            self.streaks.pop(key, None)


class TransientEvidence:
    def __init__(self, solid=None):
        self.solid=solid
        self.bounds={}
        self.solid_removed=0
        self.windows = {s:EffectiveVotes() for s in ('head','bottom')}
        self.cells = {}
        self.index = None
        if Path(__file__).with_name('libfast_rays.so').exists():
            from fast_rays import FastRays
            self.index = FastRays()
        self.tiles={}
        self.columns=Counter()
        self.cleared_xy = set()
        self.deleted = {}

    def update(self, source, ns, origin, rays, hits, pose=None):
        window = self.windows[source]
        if ns <= window.last:return
        crossed=set()
        if self.cells and len(rays):
            if self.index is not None:
                crossed=self.index.crossed(origin,rays)
            else:
                keys=list(self.cells)
                centres=(np.asarray(keys)+.5)*SIZE
                bounds=np.asarray([self.cells[k][:2] for k in keys])
                found=measured_crossings(centres,bounds,origin,rays,voxel_size=SIZE,exit_margin=.03)
                crossed={keys[i] for i in found}
        measured,inv=np.unique(np.floor(hits/SIZE).astype(int),axis=0,return_inverse=True)
        lows=np.full(len(measured),np.inf); highs=np.full(len(measured),-np.inf)
        np.minimum.at(lows,inv,hits[:,2]);np.maximum.at(highs,inv,hits[:,2])
        current=set(map(tuple,measured))
        if self.solid is not None:
            mins=np.full((len(measured),3),np.inf)
            maxs=np.full((len(measured),3),-np.inf)
            np.minimum.at(mins,inv,hits);np.maximum.at(maxs,inv,hits)
        body_clear=set()
        if self.solid is not None and pose is not None:
            rotation,translation=pose
            tile=np.floor(translation[:2]/.5).astype(int)
            near=(k for dx in range(-2,3) for dy in range(-2,3)
                  for k in self.tiles.get((tile[0]+dx,tile[1]+dy),()))
            eligible=[k for k in near if k not in current and self.cells[k][2]<ns and k in self.bounds]
            if eligible:
                lo,hi=np.asarray([self.bounds[k] for k in eligible]).transpose(1,0,2)
                mask=self.solid.contains_bounds(lo,hi,rotation,translation)
                body_clear={k for k,inside in zip(eligible,mask) if inside}
                self.solid_removed+=len(body_clear)
        # An actual endpoint in the voxel cannot count as a through-ray.
        endpoints=set(map(tuple,np.floor(rays/SIZE).astype(int))) | current
        endpoints={k for k in endpoints if k in self.cells}
        crossed.difference_update(endpoints)
        clear=window.add(ns,crossed,endpoints)
        retired={k for k in clear if k in self.cells and self.cells[k][2]<=ns}
        retired.update(body_clear)
        if self.index is not None:self.index.erase(retired)
        for key in retired:
            tile=(key[0]//10,key[1]//10); self.tiles[tile].discard(key)
            if not self.tiles[tile]:del self.tiles[tile]
            self.columns[key[:2]]-=1
            if not self.columns[key[:2]]:del self.columns[key[:2]]
            self.bounds.pop(key,None)
            del self.cells[key]
            self.deleted[key]=ns
            self.cleared_xy.add(key[:2])
        if retired:
            for history in self.windows.values():history.forget(retired)
        for i,(row,low,high) in enumerate(zip(measured,lows,highs)):
            key=tuple(row)
            if self.deleted.get(key,-1)>ns:continue
            previous=self.cells.get(key)
            if previous is not None:
                if previous[2]>ns:continue
                low=min(low,previous[0]);high=max(high,previous[1])
            if self.solid is not None:
                if key in self.bounds:
                    mn,mx=self.bounds[key]
                    self.bounds[key]=(np.minimum(mn,mins[i]),np.maximum(mx,maxs[i]))
                else:self.bounds[key]=(mins[i].copy(),maxs[i].copy())
            if previous is None:
                self.columns[key[:2]]+=1
                self.tiles.setdefault((key[0]//10,key[1]//10),set()).add(key)
            self.cells[key]=(low,high,ns)
        updated=[tuple(k) for k in measured if tuple(k) in self.cells]
        if self.index is not None and updated:self.index.set(updated,[self.cells[k][:2] for k in updated])
        if len(self.deleted)>1024:
            self.deleted={k:t for k,t in self.deleted.items() if ns-t<2_000_000_000}

    def projection(self):
        occupied=self.columns.keys()
        def points(keys,value):
            xy=(np.asarray(list(keys),float).reshape(-1,2)+.5)*SIZE[:2]
            return np.column_stack((xy,np.full(len(xy),value)))
        return np.vstack((points(self.cleared_xy-occupied,-1.),points(occupied,1.))).astype(np.float32)
