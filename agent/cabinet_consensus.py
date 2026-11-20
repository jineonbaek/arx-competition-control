"""Fresh stationary exposures, median observed red corners, no hardware access."""
import time
import numpy as np
from .cabinet_preview_geometry import detect

def fuse_cabinet_frames(frames,*,now=None):
    now=time.time() if now is None else now
    if len(frames)!=5:raise ValueError('Five independent cabinet exposures required')
    stamps=np.array([f['image_time'] for f in frames],float)
    if (not np.isfinite(stamps).all() or np.any(np.diff(stamps)<=0) or stamps[-1]-stamps[0]>1.2
            or not 0<=now-stamps[-1]<=.6 or not 0<=now-np.median(stamps)<=.8 or len({f['id'] for f in frames})!=5):
        raise ValueError('Cabinet consensus exposure identity or freshness invalid')
    last=frames[-1]
    for frame in frames:
        if any(frame[k]!=last[k] for k in ('camera_id','camera_serial','calibration_sha256','intrinsics','body_configuration')):
            raise ValueError('Cabinet consensus camera or body configuration changed')
        if (not np.allclose(frame['base_from_color'],last['base_from_color'],atol=1e-9,rtol=0)
                or np.max(np.abs(frame['joints']-last['joints']))>np.radians(.1)
                or abs(frame['lift_rad']-last['lift_rad'])>.02):
            raise ValueError('Robot moved during cabinet consensus')
    detections=[detect(f['rgb']) for f in frames]
    if any(d['status']!='complete' for d in detections):raise ValueError('Visible red edges missing in consensus exposure')
    quads=np.array([d['inner_quad_px'] for d in detections]);quad=np.median(quads,axis=0)
    spread=float(np.max(np.linalg.norm(quads-quad,axis=2)))
    if spread>1.5:raise ValueError('Cabinet marker moved or is inconsistent across exposures')
    result=dict(detections[-1],inner_quad_px=quad.tolist(),consensus_exposures=[f['id'] for f in frames],
                consensus_times=stamps.tolist(),consensus_time=float(np.median(stamps)),quad_spread_px=spread)
    return result,last
