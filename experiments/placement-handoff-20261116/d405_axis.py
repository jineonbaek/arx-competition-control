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
    rows,cols=np.nonzero(valid);camera_points=xyz[valid]
    points=camera_points@T[:3,:3].T+T[:3,3]
    if len(points)<800:raise ValueError("insufficient visible side depth")
    stride=max(1,len(points)//6000);points=points[::stride];rows=rows[::stride];cols=cols[::stride];camera_points=camera_points[::stride];rng=np.random.default_rng(47);best=None
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
    kept_rows=rows[best];kept_cols=cols[best];cam=camera_points[best]
    if len(np.unique(kept_rows))<12 or np.ptp(kept_rows)<10:raise ValueError("side extent is insufficient for direction")
    u0=float(np.median(kept_cols));v0=float(np.median(kept_rows))
    design=np.column_stack((kept_cols-u0,kept_rows-v0,np.ones(len(kept_rows))))
    rayfit,_,rank,_=np.linalg.lstsq(design,cam[:,:2]/cam[:,2,None],rcond=None)
    if rank<3 or not np.isfinite(rayfit).all():raise ValueError("side ray fit unavailable")
    normal=v[2];ends=[]
    for row in np.percentile(kept_rows,[10,90]):
        ray=np.r_[rayfit[1]*(row-v0)+rayfit[2],1.];direction=T[:3,:3]@ray;den=float(normal@direction)
        if abs(den)<.05:raise ValueError("side plane and camera ray nearly parallel")
        distance=float(normal@(center-T[:3,3])/den)
        if not .06<distance<1.5:raise ValueError("side-ray intersection invalid")
        ends.append(T[:3,3]+distance*direction)
    slope=ends[1]-ends[0];length=np.linalg.norm(slope)
    if length<.02:raise ValueError("long-axis direction unavailable")
    axis=slope/length
    if axis@(center-T[:3,3])<0:axis=-axis
    if normal@(T[:3,3]-center)<0:normal=-normal
    residual=np.abs((support-center)@normal)*1000
    return dict(status="axis_candidate",camera="d405",axis_base=axis.tolist(),side_normal_base=normal.tolist(),
        tilt_deg=float(np.degrees(np.arcsin(np.clip(abs(axis[2]),0,1)))),extent_mm=(span*1000).tolist(),
        support_points=int(len(support)),residual_p90_mm=float(np.percentile(residual,90)),executable=False,
        note="Unverified side-face hypothesis; not a calibrated motion target")

def axis_consensus(frames):
    """Compare repeated frames at one still pose; repeatability is not accuracy."""
    if not isinstance(frames,(list,tuple)) or not 3<=len(frames)<=20:raise ValueError("three to twenty D405 frames required")
    records=[];axes=[];observations=[];calibrations=[]
    for frame in frames:
        try:
            t=float(frame["image_time"]);at=float(frame["arm_time"]);bt=float(frame["body_time"]);q=np.asarray(frame["joints"],float);h=float(frame["lift_rad"]);cal=str(frame["calibration_sha256"])
        except (KeyError,TypeError,ValueError) as error:raise ValueError("frame metadata missing") from error
        if q.shape!=(6,) or len(cal)!=64 or not np.isfinite(np.r_[t,at,bt,q,h]).all() or max(abs(t-at),abs(t-bt))>.2:raise ValueError("frame metadata invalid or unpaired")
        records.append((t,q,h));calibrations.append(cal);obs=estimate_long_axis(frame);observations.append(obs);axes.append(np.asarray(obs["axis_base"],float))
    if len(set(calibrations))!=1:raise ValueError("camera calibration changed")
    if max(x[0] for x in records)-min(x[0] for x in records)>3.:raise ValueError("frame interval too long")
    if np.max(np.ptp(np.array([x[1] for x in records]),axis=0))>np.radians(.25) or np.ptp([x[2] for x in records])>.05:
        raise ValueError("robot moved between observations")
    ref=axes[0];aligned=np.array([a if a@ref>=0 else -a for a in axes]);center=np.median(aligned,axis=0);center/=np.linalg.norm(center)
    deviations=np.degrees(np.arccos(np.clip(aligned@center,-1,1)));maximum=float(np.max(deviations))
    if maximum>2.:raise ValueError("axis observations inconsistent")
    return dict(status="repeatable_axis_candidate",camera="d405",axis_base=center.tolist(),tilt_deg=float(np.degrees(np.arcsin(np.clip(abs(center[2]),0,1)))),max_deviation_deg=maximum,frame_count=len(frames),physical_accuracy_verified=False,executable=False,observations=observations)
