"""Observation-owned 3-D obstacles in odom. Time alone never clears a measured voxel."""
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import label
from scipy.spatial import cKDTree
from local_floor import LocalFloor
from grazing_floor import GrazingFloor

SIZE=np.array([.05,.05,.075])


@dataclass
class Cell:
    seen: float
    owner: int
    clears: int=0
    last_clear: float=-1.
    zmin: float=np.inf
    zmax: float=-np.inf
    bottom_seen: float=-1.
    head_seen: float=-1.
    hits: int=0
    last_hit: float=-1.
    band_times: dict=field(default_factory=dict)
    clear_bands: set=field(default_factory=set)
    floor_count: int=0
    floor_stamp: float=-1.


@dataclass
class Track:
    centre: np.ndarray
    stamp: float
    extent: np.ndarray
    trail: deque=field(default_factory=lambda:deque(maxlen=24))
    vacated: deque=field(default_factory=lambda:deque(maxlen=48))
    velocity: np.ndarray=field(default_factory=lambda:np.zeros(2))
    moving: bool=False
    latest: set=field(default_factory=set)
    evidence: dict=field(default_factory=dict)


def clusters(points):
    """Compact isolated clusters only. Large structures remain ordinary persistent voxels."""
    if len(points)<5:return []
    xy=np.floor(points[:,:2]/.14).astype(int)
    xy-=xy.min(axis=0)
    mask=np.zeros(xy.max(axis=0)+1,bool);mask[tuple(xy.T)]=True
    ids=label(mask,np.ones((3,3),int))[0][tuple(xy.T)]
    groups=[]
    for i in np.unique(ids):
        group=points[ids==i]
        if len(group)<5:continue
        size=np.ptp(group,axis=0)
        if max(size[:2])>1.05 or max(size[:2])<.10 or size[2]<.20:continue
        groups.append((group,(group[:,:2].max(0)+group[:,:2].min(0))/2,size))
    return groups


def credible(points, previous):
    if not len(points):return points
    indices=np.unique(np.floor(points/.045).astype(int),axis=0,return_index=True)[1]
    p=points[indices]
    support=cKDTree(p).query(p,k=2)[0][:,1]<.13
    if previous is not None and len(previous):
        support|=cKDTree(previous).query(p,distance_upper_bound=.085)[0]<.085
    return p[support]


def without_self_shadow(points,blockers,origin):
    if not len(blockers) or not len(points):return points
    blocked=blockers-origin
    radii=np.linalg.norm(blocked,axis=1)
    good=radii>.01;blocked=blocked[good];radii=radii[good]
    if not len(radii):return points
    rays=points-origin;distance=np.linalg.norm(rays,axis=1)
    angular,index=cKDTree(blocked/radii[:,None]).query(rays/np.maximum(distance[:,None],1e-9))
    # Around the measured body return keep a one-degree cone occluded.
    return points[(angular>np.deg2rad(1.))|(distance<radii[index]-.04)]


def ray_crossings(centres,origin,endpoints):
    if not len(centres) or not len(endpoints):return np.empty(0,int)
    v=endpoints-origin;r=np.linalg.norm(v,axis=1)
    good=(r>.35)&(r<=8.)
    v=v[good];r=r[good]
    if not len(r):return np.empty(0,int)
    # Within each small 3-D angular bin, use only the nearest finite return.
    # A ray must pass the middle of the occupied voxel and continue beyond it.
    az=np.arctan2(v[:,1],v[:,0]);el=np.arctan2(v[:,2],np.linalg.norm(v[:,:2],axis=1))
    bins=np.floor(np.c_[az,el]/np.deg2rad(.7)).astype(int)
    order=np.argsort(r)
    first=order[np.unique(bins[order],axis=0,return_index=True)[1]]
    unit=v[first]/r[first,None];length=r[first]
    delta=centres-origin;dist=np.linalg.norm(delta,axis=1)
    nearby=np.flatnonzero((dist>.35)&(dist<7.95))
    if not len(nearby):return nearby
    rel=delta[nearby]
    ids=cKDTree(unit).query(rel/dist[nearby,None],k=min(2,len(unit)))[1].reshape(len(nearby),-1)
    direction=unit[ids];along=np.sum(rel[:,None,:]*direction,axis=2)
    offset=rel[:,None,:]-along[:,:,None]*direction
    valid=(along>.35)&(along+.09<length[ids])&(np.linalg.norm(offset[:,:,:2],axis=2)<.025)&(abs(offset[:,:,2])<.035)
    return nearby[np.any(valid,axis=1)]


def measured_crossings(centres, bounds, origin, endpoints, diagnostics=None, band_counts=None, voxel_size=SIZE, exit_margin=.09):
    """Only clear measured voxels where a finite ray traverses the occupied height."""
    empty = np.empty(0, dtype=int)
    if not len(centres) or not len(endpoints):
        return empty
    vectors = endpoints - origin
    lengths = np.linalg.norm(vectors, axis=1)
    usable = (lengths > .35) & (lengths <= 8.)
    vectors, lengths = vectors[usable], lengths[usable]
    if not len(lengths):
        return empty
    az = np.arctan2(vectors[:, 1], vectors[:, 0])
    el = np.arctan2(vectors[:, 2], np.linalg.norm(vectors[:, :2], axis=1))
    bins = np.floor(np.column_stack((az, el)) / np.deg2rad(.7)).astype(int)
    order = np.argsort(lengths)
    nearest = order[np.unique(bins[order], axis=0, return_index=True)[1]]
    directions = vectors[nearest] / lengths[nearest, None]
    rel = centres - origin
    distances = np.linalg.norm(rel, axis=1)
    idx = np.flatnonzero((distances > .35) & (distances < 8.) & np.isfinite(bounds).all(axis=1))
    if not len(idx):
        return empty
    # Enclose the voxel's angular footprint at its own distance.
    radius = np.linalg.norm(voxel_size / 2)
    angles = np.arcsin(radius / distances[idx])
    candidates = cKDTree(directions).query_ball_point(
        rel[idx] / distances[idx, None], 2 * np.sin(angles / 2))
    counts = np.fromiter(map(len, candidates), int, count=len(idx))
    if not np.any(counts):
        return empty
    cells = np.repeat(idx, counts)
    beams = np.concatenate(candidates[counts > 0]).astype(int)
    direction = directions[beams]
    parallel = abs(direction) < 1e-10
    divisor = np.where(parallel, 1., direction)

    def intervals(low, high):
        first, last = low / divisor, high / divisor
        entry = np.where(parallel, -np.inf, np.minimum(first, last)).max(axis=1)
        exit_ = np.where(parallel, np.inf, np.maximum(first, last)).min(axis=1)
        intersects = ~np.any(parallel & ((low > 0) | (high < 0)), axis=1)
        return np.maximum(entry, 0.), exit_, intersects

    low, high = rel[cells] - voxel_size / 2, rel[cells] + voxel_size / 2
    enter, leave, inside = intervals(low, high)
    entered = inside & (leave > enter) & (lengths[nearest[beams]] >= enter)
    beyond = lengths[nearest[beams]] > leave + exit_margin
    # Reject corner clips. For a cell with a range of historic returns, a
    # single beam must traverse both extremes before it provides clear evidence.
    low[:, :2] += .003
    high[:, :2] -= .003
    if band_counts is not None:
        # Independent height samples no farther apart than the two-sided
        # height tolerance. A nearly horizontal beam can clear one sample.
        passed = []
        for band in range(int(band_counts[cells].max())):
            use = entered & beyond & (band_counts[cells] > band)
            if not use.any():continue
            heights = bounds[cells,0] + (bounds[cells,1]-bounds[cells,0])*band / np.maximum(1,band_counts[cells]-1)
            low[:,2]=np.maximum(rel[cells,2]-voxel_size[2]/2,heights-origin[2]-.012)
            high[:,2]=np.minimum(rel[cells,2]+voxel_size[2]/2,heights-origin[2]+.012)
            enter,leave,inside=intervals(low,high)
            passed.extend((int(j),band) for j in np.unique(cells[use & inside & (leave-enter>=.008)]))
        return passed
    measured = entered.copy()
    for height in bounds[cells].T:
        low[:, 2] = np.maximum(rel[cells, 2] - voxel_size[2]/2, height - origin[2] - .012)
        high[:, 2] = np.minimum(rel[cells, 2] + voxel_size[2]/2, height - origin[2] + .012)
        enter, leave, inside = intervals(low, high)
        measured &= inside & (leave - enter >= .008) & (lengths[nearest[beams]] >= enter + .008)
    if diagnostics is not None:
        diagnostics += [entered.sum(), measured.sum(), (measured & beyond).sum()]
    return np.unique(cells[measured & beyond])


class World:
    def __init__(self):
        self.cells={};self.previous={};self.last={};self.tracks={};self.next_id=1
        self.grazing=GrazingFloor();self.grazing_removed=0;self.grazing_filtered=0
        self.floor=LocalFloor();self.floor_removed=0;self.floor_filtered={"head":0,"bottom":0}
        self.ray_removed=self.track_removed=0
        self.legacy={};self.legacy_cursor=0
        self.last_global=-1.;self.deleted={};self.last_gc=-1.

    def associate(self,groups,t,source):
        tracks={k:v for k,v in self.tracks.items() if t-v.stamp<.65}
        self.tracks=tracks
        edges=[]
        for j,(_,centre,size) in enumerate(groups):
            for i,tr in tracks.items():
                dt=t-tr.stamp
                predicted=tr.centre+tr.velocity*np.clip(dt,0,.35)
                distance=np.linalg.norm(centre-predicted)
                if distance<(.15+1.6*max(dt,0)) and np.linalg.norm(size[:2]-tr.extent[:2])<.42:
                    edges.append((distance,j,i))
        assigned={};used=set()
        for _,j,i in sorted(edges):
            if j not in assigned and i not in used:assigned[j]=i;used.add(i)
        owner={}
        for j,(group,centre,size) in enumerate(groups):
            if j not in assigned:
                i=self.next_id;self.next_id+=1
                tracks[i]=Track(centre,t,size)
            else:i=assigned[j]
            tr=tracks[i]
            if not tr.trail or t-tr.trail[-1][0]>=.045 or tr.trail[-1][2]!=source:
                tr.trail.append((t,centre.copy(),source,size.copy()))
            samples=[(s,c,z) for s,c,src,z in tr.trail if src==source and 0<=t-s<=1.]
            tr.evidence=dict(sensor=source, samples=len(samples))
            if len(samples)>=5:
                dt=t-samples[0][0];shift=centre-samples[0][1];distance=np.linalg.norm(shift)
                travel=np.sum(np.linalg.norm(np.diff([c for _,c,_ in samples],axis=0),axis=1))
                behind={key[:2] for when,key in tr.vacated if t-when<.75 and
                        np.dot(centre-(np.asarray(key[:2])+.5)*SIZE[:2],shift)>distance*.10}
                stable_shape=np.max(np.ptp([z for _,_,z in samples],axis=0)[:2])<.22
                coherent = len(samples)>=6 and distance>.30 and distance>.93*travel
                tr.evidence.update(span_s=round(dt,3),shift_m=round(float(distance),3),
                                   travel_m=round(float(travel),3),vacated_xy=len(behind),
                                   stable_shape=bool(stable_shape),coherent=bool(coherent))
                if (dt>.30 and .20<distance<1.8*dt and distance>.80*travel
                    and (len(behind)>=3 or coherent) and stable_shape):
                    tr.moving=True;tr.velocity=.5*tr.velocity+.5*shift/dt
                elif travel<.08:
                    tr.velocity*=.5
                    if dt>.6:tr.moving=False
            tr.centre=centre;tr.extent=size;tr.stamp=t
            keys={tuple(k) for k in np.floor(group/SIZE).astype(int)}
            tr.latest=keys
            for k in keys:owner[k]=i
        return owner,used|set(owner.values())

    def floor_filter(self,source,t,origin,rays,raw_hits,ground,extra):
        self.floor.observe(source,t,origin,rays,ground)
        # Only same-frame, locally supported near-surface returns count as floor evidence.
        if extra is not None and len(extra) and len(raw_hits):
            above=extra[:,2]-self.floor.height(extra[:,:2])
            low=extra[(above>=.028)&(above<=.15)]
            # Sub-threshold returns may add vertical detail only in a column
            # already marked by an independently qualifying face in this scan.
            rh=raw_hits[:,2]-self.floor.height(raw_hits[:,:2])
            faces=raw_hits[(rh>=.050)&(rh<=.35)]
            if len(low) and len(faces):
                columns={}
                for key,z in zip(np.floor(faces[:,:2]/SIZE[:2]).astype(int),faces[:,2]):
                    k=tuple(key);columns[k]=max(columns.get(k,-np.inf),z)
                supported=[p for p,k in zip(low,np.floor(low[:,:2]/SIZE[:2]).astype(int))
                           if .025<=columns.get(tuple(k),-np.inf)-p[2]<=.15]
                if supported:raw_hits=np.vstack((raw_hits,supported))
        candidates=np.vstack((rays,raw_hits))
        if ground is not None:candidates=np.vstack((candidates,ground))
        if not len(candidates):return raw_hits
        h=self.floor.height(candidates[:,:2])
        dz=candidates[:,2]-h
        lo=(dz>=-.035)&(dz<=.022)
        speckle=self.grazing.classify(t,candidates,dz,len(rays)) if source=='head' else np.zeros(len(dz),bool)
        self.grazing_filtered+=int(speckle[len(rays):len(rays)+len(raw_hits)].sum())
        upright=(dz>.022)&(dz<=.20)&~speckle
        if np.any(lo) and np.any(upright):
            near=cKDTree(candidates[upright,:2]).query(candidates[lo,:2],distance_upper_bound=.08)[0]
            lo[np.flatnonzero(lo)[np.isfinite(near)]]=False
        mask=(lo|speckle)[len(rays):len(rays)+len(raw_hits)]
        self.floor_filtered[source]+=int(mask.sum())
        # The actual historic height range matters: a 7.5 cm voxel can contain
        # both floor and a low obstacle. Never erase it on its centre height alone.
        evidence=lo|speckle
        if np.any(evidence):
            points=candidates[evidence]
            keys=np.floor(points/SIZE).astype(int)
            unique,first=np.unique(keys,axis=0,return_index=True)
            levels=h[evidence][first]
            mixed_keys={tuple(k) for k in np.floor(candidates[speckle]/SIZE).astype(int)}
            for key,level in zip(unique,levels):
                for k_z in (-1,0,1):
                    k=(key[0],key[1],key[2]+k_z)
                    c=self.cells.get(k)
                    if c is None or t<=c.seen or t-c.floor_stamp<.075:continue
                    mixed=(tuple(key) in mixed_keys and c.head_seen>=0 and c.bottom_seen<0)
                    # Two observed bare-floor frames may contradict a single
                    # near-surface hit. Repeated vertical features keep the stricter gate.
                    weak = c.hits == 1 and 0 < t-c.seen < 2.
                    ceiling = .055 if mixed else .035 if weak else .022
                    if not level-.035<=c.zmin<=c.zmax<=level+ceiling:continue
                    c.floor_count=c.floor_count+1 if t-c.floor_stamp<.8 else 1
                    c.floor_stamp=t
                    if c.floor_count >= (2 if weak else 3):
                        del self.cells[k];self.deleted[k]=t;self.floor_removed+=1
                        if mixed:self.grazing_removed+=1
        if source=='head' and len(self.grazing.verified):
            # Sparse scans may take seconds to revisit a single cell. Use the
            # independent exact-column observations in the verified window.
            xy=self.grazing.verified
            floors=self.floor.height((xy+.5)*SIZE[:2])
            for col,level in zip(xy,np.asarray(floors)):
                if not np.isfinite(level):continue
                low=int(np.floor((level-.035)/SIZE[2]))
                high=int(np.floor((level+.055)/SIZE[2]))
                for z in range(low,high+1):
                    k=(col[0],col[1],z);c=self.cells.get(k)
                    if c is None or c.head_seen<0 or c.bottom_seen>=0 or c.seen>=t:continue
                    if level-.035<=c.zmin<=c.zmax<=level+.055:
                        del self.cells[k];self.deleted[k]=t
                        self.floor_removed+=1;self.grazing_removed+=1
        # An unsupported patch inherits the original conservative sensor height gate.
        rh=h[len(rays):len(rays)+len(raw_hits)]
        delta=raw_hits[:,2]-rh
        upper=1.32 if source=='head' else 1.35
        use=~np.isfinite(rh)|((delta>=-.035)&(delta<=upper))
        return raw_hits[use&~mask]

    def check_legacy(self,origin,rays,raw_hits,t):
        # The pre-upgrade public map carried XY only. Preserve unresolved
        # heights until distinct finite rays cross each possible height bin.
        if not self.legacy:return
        keys=list(self.legacy)
        subset=(keys[self.legacy_cursor:self.legacy_cursor+200] or keys[:200])
        self.legacy_cursor=(self.legacy_cursor+len(subset))%len(keys)
        cols=np.asarray(subset,int)
        ij=np.column_stack((np.repeat(cols,18,axis=0),np.tile(np.arange(18),len(cols))))
        centers=(ij+.5)*SIZE
        crossed=ray_crossings(centers,origin,rays)
        veto={tuple(k) for k in np.floor(raw_hits/SIZE).astype(int)}
        for i in crossed:
            key=tuple(ij[i,:2]);z=int(ij[i,2]);mask,stamp=self.legacy[key]
            if tuple(ij[i]) in veto:stamp[z]=-1.;continue
            if stamp[z]>=0 and .055<=t-stamp[z]<2.:mask[z]=False
            stamp[z]=t
        for key in subset:
            if not self.legacy[key][0].any():del self.legacy[key]

    def inspect(self,xy,now):
        index=tuple(np.floor(np.asarray(xy)/SIZE[:2]).astype(int))
        floor=self.floor.height(np.array([xy]))[0]
        rows=[]
        for key,c in self.cells.items():
            if key[:2]!=index:continue
            rows.append(dict(voxel_z=int(key[2]),odom_z=[float(c.zmin),float(c.zmax)],
                height_above_floor=[float(c.zmin-floor),float(c.zmax-floor)] if np.isfinite(floor) else None,
                track_id=int(c.owner), track_moving=bool(c.owner in self.tracks and self.tracks[c.owner].moving),
                track_evidence=self.tracks[c.owner].evidence if c.owner in self.tracks else None,
                hit_frames=c.hits, height_bands_cleared=len(c.clear_bands),
                last_seen=float(c.seen),age_s=round(now-c.seen,2),
                needs_reobservation=now-c.seen>2.,
                recent_clear_frames=int(c.clears) if 0<=now-c.last_clear<2. else 0,
                sources={s:float(t) for s,t in [('bottom',c.bottom_seen),('head',c.head_seen)] if t>=0}))
        predicted=not rows and index not in self.legacy and any(np.all(np.floor(p/SIZE[:2]).astype(int)==index) for p in self.projection(now))
        return dict(odom_xy=list(map(float,xy)),legacy_height_unknown=index in self.legacy,local_floor_z=float(floor) if np.isfinite(floor) else None,
                    prediction_only=bool(predicted),voxels=sorted(rows,key=lambda row:row['voxel_z']))

    def update(self,source,t,origin,rays,raw_hits,ground=None,extra=None):
        if t<=self.last.get(source,-1) or t<self.last_global-.45:return False
        late=t<self.last_global
        if t-self.last_gc>1:
            self.deleted={k:v for k,v in self.deleted.items() if t-v<1.}
            self.last_gc=t
        self.check_legacy(origin,rays,raw_hits,t)
        raw_hits=self.floor_filter(source,t,origin,rays,raw_hits,ground,extra)
        prev=self.previous.get(source) if t-self.last.get(source,-1)<.35 else None
        hits=credible(raw_hits,prev);self.previous[source]=raw_hits
        self.last[source]=t;self.last_global=max(self.last_global,t)
        ids={tuple(k) for k in np.floor(hits/SIZE).astype(int)}
        voxel,indices=np.unique(np.floor(raw_hits/SIZE).astype(int),axis=0,return_inverse=True)
        low=np.full(len(voxel),np.inf);high=np.full(len(voxel),-np.inf)
        np.minimum.at(low,indices,raw_hits[:,2]);np.maximum.at(high,indices,raw_hits[:,2])
        bounds={tuple(k):(mn,mx) for k,mn,mx in zip(voxel,low,high)}
        current_returns=bounds.keys()
        if late:
            # Do not rewind track state when head registration arrives after the bottom packet.
            owner={};updated=set()
            for group,centre,_ in clusters(hits):
                options=[(np.linalg.norm(centre-(tr.centre+tr.velocity*(t-tr.stamp))),i)
                         for i,tr in self.tracks.items() if abs(t-tr.stamp)<.45]
                if options and min(options)[0]<.24:
                    i=min(options)[1]
                    for k in np.floor(group/SIZE).astype(int):owner[tuple(k)]=i
        else:owner,updated=self.associate(clusters(hits),t,source)
        # An existing background observation or ambiguous ownership is protected.
        for k in ids:
            who=owner.get(k,0)
            cell=self.cells.get(k)
            if self.deleted.get(k,-1)>=t or (cell and cell.seen>t):continue
            tr=self.tracks.get(who)
            if late and tr and tr.moving and np.any(abs((np.asarray(k[:2])+.5)*SIZE[:2]-tr.centre)>tr.extent[:2]/2+.075):continue
            if cell is not None and cell.owner!=who:who=0
            if cell is None:
                cell=Cell(t,who);self.cells[k]=cell
            cell.owner=who
        for k,(mn,mx) in bounds.items():
            c=self.cells.get(k)
            if c is None:continue
            if t-c.last_hit>=.055:
                c.hits+=1;c.last_hit=t
            c.band_times.clear();c.clear_bands.clear()
            c.zmin=min(c.zmin,mn);c.zmax=max(c.zmax,mx)
            c.seen=max(t,c.seen);c.clears=0;c.floor_count=0
            if source=='head':c.head_seen=max(t,c.head_seen)
            else:c.bottom_seen=max(t,c.bottom_seen)
        for k in current_returns-ids:
            c=self.cells.get(k)
            if c is not None and t>=c.seen:
                c.seen=t;c.clears=0
        if self.cells:
            ray_end_cells={tuple(k) for k in np.floor(rays/SIZE).astype(int)}
            for k in ray_end_cells:
                c=self.cells.get(k)
                if c is not None and t>=c.seen:
                    c.clears=0;c.band_times.clear();c.clear_bands.clear()
            keys=list(self.cells)
            centres=(np.asarray(keys)+.5)*SIZE
            bounds=np.array([(self.cells[k].zmin,self.cells[k].zmax) for k in keys])
            bands = 1+np.ceil(np.maximum(0.,bounds[:,1]-bounds[:,0])/.022).astype(int)
            for j,band in measured_crossings(centres,bounds,origin,rays,band_counts=bands):
                k=keys[j]
                if k not in self.cells:continue
                c=self.cells[k]
                if k in current_returns or k in ray_end_cells or t-c.seen<.075:
                    continue
                stamps=c.band_times.setdefault(band,[])
                stamps[:]=[stamp for stamp in stamps if 0<=t-stamp<2.]
                if not stamps or t-stamps[-1]>=.055:
                    stamps.append(t)
                    del stamps[:-2]
                if len(stamps)>=2:c.clear_bands.add(band)
                c.clears=min(2 if i in c.clear_bands else sum(0<=t-stamp<2. for stamp in c.band_times.get(i,[])) for i in range(bands[j]))
                c.last_clear=t
                if c.clears>=2:
                    tr=self.tracks.get(c.owner)
                    if tr:tr.vacated.append((t,k))
                    del self.cells[k];self.deleted[k]=t;self.ray_removed+=1
            moving={i for i in updated if self.tracks[i].moving}
            for k,c in (list(self.cells.items()) if moving else ()):
                if c.owner not in moving:continue
                tr=self.tracks.get(c.owner)
                if not tr or not tr.moving or c.owner not in updated or k in tr.latest or k in current_returns:continue
                # Only retire this track's own old footprint, after it has moved away.
                pos=(np.asarray(k[:2])+.5)*SIZE[:2]
                if np.any(abs(pos-tr.centre)>tr.extent[:2]/2+.075):
                    del self.cells[k];self.deleted[k]=t;self.track_removed+=1
        return True

    def projection(self,now):
        xy={k[:2] for k in self.cells}|set(self.legacy)
        for tr in self.tracks.values():
            age=now-tr.stamp
            if not tr.moving or not .10<age<.35:continue
            offset=np.rint(tr.velocity*age/SIZE[:2]).astype(int)
            # Conservative prediction adds only the current estimated footprint.
            xy.update((k[0]+offset[0],k[1]+offset[1]) for k in tr.latest)
        return (np.array(list(xy),float).reshape(-1,2)+.5)*SIZE[:2]
