"""Candidate D405 battery side long-axis estimator. Never commands a robot."""
import cv2
import numpy as np

def estimate_long_axis(frame):
    rgb=np.asarray(frame["rgb"]);xyz=np.asarray(frame["xyz"],float);T=np.asarray(frame["base_from_color"],float)
    if (str(frame["camera_id"])!="d405" or rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or xyz.shape!=rgb.shape
        or T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1])
        or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5)):
        raise ValueError("invalid D405 frame or transform")
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);dark=(hsv[:,:,2]<100).astype(np.uint8);h,w=dark.shape
    n,labels,stats,centers=cv2.connectedComponentsWithStats(dark,8)
    candidates=[i for i in range(1,n) if stats[i,4]>.02*h*w]
    if not candidates:raise ValueError("battery silhouette unavailable")
    idx=min(candidates,key=lambda i:np.linalg.norm((centers[i]-[w/2,h*.45])/[w,h]))
    x,y,bw,bh=map(int,stats[idx,:4]);region=np.zeros((h,w),bool)
    region[y:min(h,y+max(1,int(.22*bh))),x+int(.30*bw):x+int(.70*bw)]=True
    valid=(labels==idx)&region&np.isfinite(xyz).all(2)&(xyz[:,:,2]>.06)&(xyz[:,:,2]<1.5)
    rows=np.nonzero(valid)[0]
    points=xyz[valid]@T[:3,:3].T+T[:3,3]
    if len(points)<800:raise ValueError("insufficient visible side depth")
    stride=max(1,len(points)//6000);points=points[::stride];rows=rows[::stride];rng=np.random.default_rng(47);best=None
    for _ in range(180):
        a,b,c=points[rng.choice(len(points),3,replace=False)];n=np.cross(b-a,c-a);norm=np.linalg.norm(n)
        if norm<1e-8:continue
        n/=norm
        if abs(n[2])>.4:continue
        mask=np.abs((points-a)@n)<.003
        if best is None or mask.sum()>best.sum():best=mask
    if best is None or best.sum()<500:raise ValueError("side plane unavailable")
    support=points[best];center=support.mean(0);_,sv,v=np.linalg.svd(support-center,full_matrices=False)
    if sv[1]<=1e-9 or sv[0]/sv[1]<1.15:raise ValueError("long-axis orientation ambiguous")
    coordinates=(support-center)@v[:2].T;span=np.ptp(coordinates,axis=0)
    if min(span)<.02 or max(span)>.25:raise ValueError("visible surface dimensions implausible")
    kept=rows[best]; unique=np.unique(kept);centers=[];ys=[]
    for row in unique:
        samples=support[kept==row]
        if len(samples)>=3:centers.append(np.median(samples,axis=0));ys.append(float(row))
    if len(ys)<12 or np.ptp(ys)<10:raise ValueError("side extent is insufficient for direction")
    design=np.column_stack((np.asarray(ys)-np.mean(ys),np.ones(len(ys))))
    slope=np.linalg.lstsq(design,np.asarray(centers),rcond=None)[0][0]
    length=np.linalg.norm(slope)
    if length<1e-6:raise ValueError("long-axis direction unavailable")
    axis=slope/length;normal=v[2]
    if axis@(center-T[:3,3])<0:axis=-axis
    if normal@(T[:3,3]-center)<0:normal=-normal
    residual=np.abs((support-center)@normal)*1000
    return dict(status="axis_candidate",camera="d405",axis_base=axis.tolist(),side_normal_base=normal.tolist(),
        tilt_deg=float(np.degrees(np.arcsin(np.clip(abs(axis[2]),0,1)))),extent_mm=(span*1000).tolist(),
        support_points=int(len(support)),residual_p90_mm=float(np.percentile(residual,90)),executable=False,
        note="Unverified side-face hypothesis; not a calibrated motion target")
