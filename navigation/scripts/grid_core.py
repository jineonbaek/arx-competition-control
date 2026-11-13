"""Conservative projection of a gravity-aligned point map; no convex-hull fill."""
import json
from pathlib import Path
import numpy as np
import yaml


def read_pcd(path):
    with Path(path).open('rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            if not line:
                raise ValueError('Incomplete PCD header')
            parts = line.split()
            header[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break
        if header['DATA'] != ['binary']:
            raise ValueError('Only PCL binary PCD supported')
        kinds = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype = np.dtype([(name, '<' + kinds[kind] + size, (int(count),)) for name, size, kind, count in
                          zip(header['FIELDS'], header['SIZE'], header['TYPE'], header['COUNT'])])
        values = np.frombuffer(f.read(), dtype=dtype, count=int(header['POINTS'][0]))
        return np.column_stack([values[k].ravel() for k in ('x', 'y', 'z')])


def fit_floor(points):
    """Estimate a near-horizontal plane below the initial sensor. Must be reviewed."""
    pts = np.asarray(points)
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[(pts[:, 2] < -0.1) & (pts[:, 2] >= np.percentile(pts[:, 2], 1))]
    if len(pts) < 100:
        raise ValueError('Insufficient floor evidence; specify --floor a b c explicitly')
    rng = np.random.default_rng(4)
    if len(pts) > 12000:
        pts = pts[rng.choice(len(pts), 12000, replace=False)]
    A = np.column_stack([pts[:, :2], np.ones(len(pts))])
    best = None
    best_count = 0
    for _ in range(400):
        idx = rng.choice(len(pts), 3, replace=False)
        if np.linalg.cond(A[idx]) > 1000:
            continue
        abc = np.linalg.solve(A[idx], pts[idx, 2])
        if np.linalg.norm(abc[:2]) > 0.1 or abc[2] > -0.1:
            continue
        mask = abs(A @ abc - pts[:, 2]) < 0.04
        if mask.sum() > best_count:
            best, best_count = mask, int(mask.sum())
    if best_count < 100 or best_count < len(pts) * 0.08:
        raise ValueError('No sufficiently supported horizontal floor; manual plane required')
    abc = np.linalg.lstsq(A[best], pts[best, 2], rcond=None)[0]
    span = np.ptp(pts[best, :2], axis=0)
    if min(span) < 1.0:
        raise ValueError('Floor support too narrow')
    return abc, {'floor_plane_z_ax_by_c': abc.tolist(), 'inliers': best_count,
                 'candidate_points': len(pts), 'support_xy_span_m': span.tolist(),
                 'rms_m': float(np.sqrt(np.mean((A[best] @ abc - pts[best, 2])**2))),
                 'floor_source': 'automatic; requires physical validation', 'navigation_validated': False}


def line_cells(x0, y0, x1, y1):
    dx, dy = abs(x1-x0), -abs(y1-y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx+dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e = 2*err
        if e >= dy:
            err += dy
            x0 += sx
        if e <= dx:
            err += dx
            y0 += sy


def project(points, floor, resolution=0.1, origins=None, min_height=0.10, max_height=2.0):
    pts = np.asarray(points)
    pts = pts[np.isfinite(pts).all(axis=1)]
    height = pts[:, 2] - (pts[:, 0]*floor[0] + pts[:, 1]*floor[1] + floor[2])
    support = abs(height) <= 0.08
    obstacle = (height >= min_height) & (height <= max_height)
    relevant = pts[support | obstacle]
    if len(relevant) < 10 or resolution <= 0:
        raise ValueError('No valid map points or invalid resolution')
    lo = np.floor((relevant[:, :2].min(axis=0)-0.5)/resolution)*resolution
    shape = np.ceil((relevant[:, :2].max(axis=0)+0.5-lo)/resolution).astype(int)+1
    if shape.prod() > 4_000_000:
        raise ValueError('Grid bounds exceed limit; refusing potentially divergent map')
    grid = np.full((shape[1], shape[0]), -1, dtype=np.int8)
    def cells(xy):
        return np.floor((xy-lo)/resolution).astype(int)
    obstacles = cells(pts[obstacle, :2])
    ground = np.unique(cells(pts[support, :2]), axis=0)
    if len(obstacles):
        grid[obstacles[:, 1], obstacles[:, 0]] = 100
    # Only observed ground establishes free space. Ray clearing terminates at
    # projected obstacles; unseen exterior cells remain unknown.
    if origins is not None and len(origins):
        from scipy.spatial import cKDTree
        origins = np.asarray(origins)[:, :2]
        start = cells(origins)
        nearest = cKDTree(origins).query(lo+(ground+0.5)*resolution)[1]
        for end, begin in zip(ground, start[nearest]):
            for x, y in line_cells(*begin, *end):
                if not (0 <= x < shape[0] and 0 <= y < shape[1]):
                    continue
                if grid[y, x] == 100:
                    break
                grid[y, x] = 0
    for x, y in ground:
        if grid[y, x] != 100:
            grid[y, x] = 0
    return grid, lo


def save_grid(prefix, grid, origin, resolution, metadata):
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    image = np.where(grid < 0, 205, np.where(grid >= 65, 0, 254)).astype(np.uint8)
    pgm = prefix.with_suffix('.pgm')
    tmp = pgm.with_suffix('.pgm.tmp')
    with tmp.open('wb') as f:
        f.write(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode())
        f.write(image[::-1].tobytes())
    tmp.replace(pgm)
    doc = {'image': pgm.name, 'mode': 'trinary', 'resolution': float(resolution),
           'origin': [float(origin[0]), float(origin[1]), 0.0], 'negate': 0,
           'occupied_thresh': 0.65, 'free_thresh': 0.196}
    prefix.with_suffix('.yaml').write_text(yaml.safe_dump(doc, sort_keys=False))
    metadata = dict(metadata, width=grid.shape[1], height=grid.shape[0],
                    free_cells=int((grid == 0).sum()), occupied_cells=int((grid == 100).sum()),
                    unknown_cells=int((grid < 0).sum()), resolution_m=resolution,
                    note='Experimental projected grid. Unknown remains unknown; review floor, obstacle height and robot footprint before navigation.')
    prefix.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
