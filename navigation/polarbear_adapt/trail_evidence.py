"""Same-frame evidence for retiring the measured trail of one compact moving cluster."""
from collections import defaultdict

import numpy as np
from scipy.spatial import cKDTree

from shared_voxels import World, clusters


EMPTY = np.empty((0, 3), dtype=np.float32)
CELL = .05


def visible_old_centres(centres, origin, rays, patch_support=None):
    """Require distinct through-beams in the old height band. A nearer return vetoes."""
    if not len(centres) or not len(rays):
        return np.zeros(len(centres), dtype=bool)
    vectors = rays - origin
    lengths = np.linalg.norm(vectors, axis=1)
    use = (lengths > .35) & (lengths <= 8.)
    if not np.any(use):
        return np.zeros(len(centres), dtype=bool)
    lengths = lengths[use]
    unit = vectors[use] / lengths[:, None]
    az = np.arctan2(unit[:, 1], unit[:, 0])
    el = np.arcsin(np.clip(unit[:, 2], -1., 1.))
    bins = np.floor(np.column_stack((az, el)) / np.deg2rad(.7)).astype(int)
    order = np.argsort(lengths)
    idx = order[np.unique(bins[order], axis=0, return_index=True)[1]]
    unit, lengths = unit[idx], lengths[idx]
    delta = centres - origin
    depth = np.linalg.norm(delta, axis=1)
    tree = cKDTree(unit)
    # Only a measured patch belonging to the same track can corroborate its
    # neighbours: do not expand the visibility band for a lone historical return.
    if patch_support is None:
        patch_support = np.zeros(len(centres), dtype=bool)
    radius = np.where(patch_support, .15, .085)
    height = np.where(patch_support, .08, .055)
    neighbours = tree.query_ball_point(delta / depth[:, None], np.hypot(radius,height)/depth)
    result = np.zeros(len(centres), dtype=bool)
    for i, ids in enumerate(neighbours):
        if len(ids) < 2:
            continue
        d = unit[ids]
        along = d @ delta[i]
        miss = d * along[:, None] - delta[i]
        near = (np.linalg.norm(miss[:, :2], axis=1) <= radius[i]) & (abs(miss[:, 2]) <= height[i])
        margin = lengths[ids] - along
        result[i] = (np.count_nonzero(near & (margin > .12)) >= 2
                     and not np.any(near & (margin <= .06)))
    return result


class SensorTrail:
    def __init__(self):
        self.world = World()
        self.owned = {}
        self.last = None
        self.background = {}
        self.last_prune = 0.
        self.alignment = None

    def observe_alignment(self, rotation, translation):
        """Invalidate ownership across a discrete localization correction."""
        if self.alignment is not None:
            previous_rotation, previous_translation = self.alignment
            cosine = (np.trace(previous_rotation.T @ rotation) - 1.) / 2
            if (np.linalg.norm(translation - previous_translation) > .08
                    or cosine < np.cos(np.deg2rad(2.))):
                self.world.tracks.clear()
                self.owned.clear()
                self.background.clear()
        self.alignment = (rotation.copy(), translation.copy())

    def evidence(self, stamp, source, origin, rays, hits):
        if self.last is not None and stamp <= self.last:
            return np.zeros(len(hits), dtype=bool), EMPTY
        if self.last is not None and stamp - self.last > .65:
            self.world.tracks.clear()
            self.owned.clear()
        self.last = stamp
        # Static room structures can join the walker into a single large cluster.
        # Keep marking them in STVL, but exclude repeatedly occupied bins from tracking.
        bins, inverse = np.unique(np.floor(hits / .16).astype(int), axis=0, return_inverse=True)
        foreground = np.ones(len(bins), dtype=bool)
        for j, coarse in enumerate(bins):
            key = tuple(coarse)
            first, last, count = self.background.get(key, (stamp, stamp, 0))
            established = count >= 6 and last - first >= 1.2
            if stamp - last > (15. if established else .8):
                first, count = stamp, 0
            self.background[key] = (first, stamp, count + 1)
            foreground[j] = stamp - first < 1.2 or count < 5
        if stamp - self.last_prune > 2.:
            self.background = {k: v for k,v in self.background.items() if stamp-v[1] <= 15.}
            self.last_prune = stamp
        groups = clusters(hits[foreground[inverse]])
        # Crossing, merging or splitting targets can make a nearest match ambiguous.
        matches = defaultdict(int)
        blocked = set()
        for _, centre, extent in groups:
            options = []
            for ident, tr in self.world.tracks.items():
                dt = stamp - tr.stamp
                if dt >= .65:
                    continue
                predicted = tr.centre + tr.velocity * min(dt, .35)
                if (np.linalg.norm(centre - predicted) < .15 + 1.6 * dt
                        and np.linalg.norm(extent[:2] - tr.extent[:2]) < .42):
                    options.append(ident)
                    matches[ident] += 1
            if len(options) > 1:
                blocked.update(options)
        blocked.update(i for i, n in matches.items() if n > 1)
        for i in blocked:
            del self.world.tracks[i]
        self.world.associate(groups, stamp, source)
        by_point = {}
        boxes = {}
        for group, centre, size in groups:
            found = [i for i, tr in self.world.tracks.items()
                     if tr.stamp == stamp and np.array_equal(tr.centre, centre)]
            if len(found) != 1:
                continue
            ident = found[0]
            boxes[ident] = (group[:, :2].min(0), group[:, :2].max(0))
            for point in group:
                by_point[tuple(point)] = ident
        keys = np.floor(hits / CELL).astype(int)
        owners = defaultdict(set)
        for point, key in zip(hits, keys):
            owners[tuple(key)].add(by_point.get(tuple(point), 0))
        for key, owners_now in owners.items():
            ident = next(iter(owners_now)) if len(owners_now) == 1 else 0
            previous, first, last = self.owned.get(key, (ident, stamp, stamp))
            if previous != ident or stamp - first > 1.2:
                ident = 0
            self.owned[key] = (ident, first, stamp)
        eligible = np.array([self.owned[tuple(k)][0] != 0 for k in keys], dtype=bool)
        moving = {}
        for ident in boxes:
            tr = self.world.tracks[ident]
            trail = [(t,p) for t,p,_,_ in tr.trail if 0 <= stamp-t <= .65]
            if len(trail) < 5 or stamp-trail[0][0] < .30:
                continue
            path = np.array([p for _,p in trail])
            shift = path[-1]-path[0]
            distance = np.linalg.norm(shift)
            travel = np.linalg.norm(np.diff(path,axis=0),axis=1).sum()
            recent = [p for t,p in trail if stamp-t <= .35]
            # Walking changes the outline with each step. Check actual progress
            # both across the track and in the most recent part of that track.
            if (.22 <= distance <= 1.8*(stamp-trail[0][0]) and distance >= .79*travel
                    and len(recent) >= 3 and np.dot(recent[-1]-recent[0],shift) > .09*distance):
                moving[ident] = shift/distance
        candidates = []
        identifiers = []
        for key, (ident, first, last) in list(self.owned.items()):
            if stamp - last > 3.0:
                del self.owned[key]
                continue
            if not ident or key in owners or ident not in moving:
                continue
            tr = self.world.tracks[ident]
            centre = (np.asarray(key) + .5) * CELL
            lo, hi = boxes[ident]
            if np.all(centre[:2] >= lo - .10) and np.all(centre[:2] <= hi + .10):
                continue
            if np.dot(tr.centre - centre[:2], moving[ident]) <= .12:
                continue
            candidates.append(centre)
            identifiers.append(ident)
        if not candidates:
            return eligible, EMPTY
        candidates = np.array(candidates)
        identifiers = np.array(identifiers)
        if len(hits):
            isolated = cKDTree(hits).query(candidates)[0] > .12
            candidates,identifiers = candidates[isolated],identifiers[isolated]
        patch = np.zeros(len(candidates),dtype=bool)
        for ident in np.unique(identifiers):
            mask = identifiers==ident
            # Neighbours must themselves be measured, vacated cells at similar height.
            neighbours = cKDTree(candidates[mask]/[.15,.15,.08])
            patch[mask] = neighbours.query_ball_point(neighbours.data,1.,return_length=True)>=4
        visible = visible_old_centres(candidates, origin, rays, patch)
        return eligible, candidates[visible]


def append_trail_evidence(message, owned, vacated):
    """Tag existing hits (5) and same-frame visible retreat candidates (6)."""
    if not len(owned) and not len(vacated):
        return
    extra = np.empty((len(owned) + len(vacated), 4), dtype='<f4')
    extra[:len(owned), :3], extra[:len(owned), 3] = owned, 5
    extra[len(owned):, :3], extra[len(owned):, 3] = vacated, 6
    message.data = bytes(message.data) + extra.tobytes()
    message.width += len(extra)
    message.row_step = message.width * message.point_step
