"""Estimate a near-horizontal dark surface in one D405 RGB-D frame; no control."""
import cv2
import numpy as np

def estimate_battery_plane(frame):
    rgb=np.asarray(frame['rgb']);xyz=np.asarray(frame['xyz'],float);T=np.asarray(frame['base_from_color'],float)
    if str(frame['camera_id'])!='d405' or rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or xyz.shape!=rgb.shape or T.shape!=(4,4) or not np.isfinite(T).all():
        raise ValueError('invalid D405 RGB-D frame')
    if not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5):raise ValueError('invalid D405 transform')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);mask=(hsv[:,:,2]<100).astype(np.uint8)
    n,labels,stats,centers=cv2.connectedComponentsWithStats(mask,8);h,w=mask.shape
    choices=[i for i in range(1,n) if stats[i,4]>.02*h*w]
    if not choices:raise ValueError('dark body component unavailable')
    idx=min(choices,key=lambda i:np.linalg.norm((centers[i]-[w/2,h*.45])/[w,h]))
    x,y,bw,bh=map(int,stats[idx,:4])
    # A single dark component can join the gripper and several battery faces.
    # Inspect its leading image segment separately; this remains only a surface
    # hypothesis until validated against known poses.
    region=np.zeros((h,w),bool)
    region[y:min(h,y+max(1,int(.30*bh))),x+int(.15*bw):x+int(.85*bw)]=True
    valid=(labels==idx)&region&np.isfinite(xyz).all(2)&(xyz[:,:,2]>.06)&(xyz[:,:,2]<1.5)
    points=xyz[valid]@T[:3,:3].T+T[:3,3]
    if len(points)<800:raise ValueError('insufficient body depth')
    points=points[::max(1,len(points)//8000)];rng=np.random.default_rng(19);best=None
    for _ in range(180):
        a,b,c=points[rng.choice(len(points),3,replace=False)];normal=np.cross(b-a,c-a);norm=np.linalg.norm(normal)
        if norm<1e-8:continue
        normal/=norm
        if abs(normal[2])<np.cos(np.radians(30.)):continue
        inside=np.abs((points-a)@normal)<.004
        if best is None or inside.sum()>best.sum():best=inside
    if best is None or best.sum()<500:raise ValueError('near-horizontal surface unavailable')
    selected=points[best];center=selected.mean(0);_,_,vh=np.linalg.svd(selected-center,full_matrices=False);normal=vh[-1]
    if normal[2]<0:normal=-normal
    projected=(selected-center)@vh[:2].T
    if min(np.ptp(projected,axis=0))<.02:raise ValueError('surface extent insufficient')
    residual=np.abs((selected-center)@normal)*1000
    return dict(status='surface_candidate',camera='d405',normal_base=normal.tolist(),tilt_deg=float(np.degrees(np.arccos(np.clip(normal[2],-1,1)))),support_points=int(len(selected)),residual_p90_mm=float(np.percentile(residual,90)),executable=False,note='plane identity and physical accuracy not verified')
