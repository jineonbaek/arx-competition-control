"""Track a locked, depth-checked static rim using fresh color corner positions."""
import cv2,json,numpy as np
from .cabinet_rgb_geometry import camera_matrix
from .placement_geometry import rigid

def marker_drift_mm(window,frame,quad):
    if str(frame['calibration_sha256'])!=window['calibration_sha256']:raise ValueError('Marker camera binding changed')
    quad=np.asarray(quad,float)
    if quad.shape!=(4,2) or not np.isfinite(quad).all():raise ValueError('Four observed marker corners required')
    k=json.loads(str(frame['intrinsics']));K,dist=camera_matrix(k,frame['rgb'].shape);T=rigid(frame['base_from_color'])
    xy=cv2.undistortPoints(quad.reshape(-1,1,2),K,dist).reshape(-1,2)
    rays=np.column_stack((xy,np.ones(4)))@T[:3,:3].T
    normal=np.asarray(window['outward_normal_base']);center=np.asarray(window['center_base_m']);den=rays@normal
    if np.min(abs(den))<.05:raise ValueError('Marker plane is grazing camera')
    scale=((center-T[:3,3])@normal)/den
    if np.min(scale)<=0:raise ValueError('Marker moved behind camera')
    points=T[:3,3]+rays*scale[:,None]
    return float(np.max(np.linalg.norm(points-np.asarray(window['corners_base_m']),axis=1))*1000)
