"""Single red ring + measured support plane. Metric outputs, no hardware I/O."""
import time
import cv2
import numpy as np

DEFAULTS=dict(battery_size_m=[.08235,.075,.19528],edge_margin_mm=10.,gap_mm=20.,
    xy_tolerance_mm=30.,z_tolerance_mm=3.,yaw_tolerance_deg=8.,max_frame_age_s=2.,
    max_plane_tilt_deg=10.,plane_residual_mm=5.,ring_residual_mm=10.)


def rigid(value):
    t=np.asarray(value,float)
    if (t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1])
        or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(t[:3,:3]),1)):
        raise ValueError('无效的米制刚性变换')
    return t


def policy(values=None):
    out=dict(DEFAULTS)
    if values:
        if set(values)-set(out):raise ValueError('未知放置容差配置')
        out.update(values)
    size=np.asarray(out['battery_size_m'],float)
    if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0) or np.any(size>1):raise ValueError('电池尺寸须为米')
    for k,v in out.items():
        if k!='battery_size_m' and (type(v) not in (float,int) or not np.isfinite(v) or v<=0):raise ValueError('放置配置须为有限正数')
    if out['z_tolerance_mm']>3 or out['xy_tolerance_mm']>30 or out['yaw_tolerance_deg']>8 or out['max_plane_tilt_deg']>10 or out['max_frame_age_s']>2:
        raise ValueError('放置配置超出已声明的软件边界')
    return out


def _plane(points, tolerance):
    if len(points)<100:raise ValueError('支撑面深度不足')
    pts=points[::max(1,len(points)//2500)];rng=np.random.default_rng(7);best=None
    for _ in range(60):
        a,b,c=pts[rng.choice(len(pts),3,replace=False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-9:continue
        n/=length;mask=np.abs((pts-a)@n)<=tolerance
        if best is None or mask.sum()>best.sum():best=mask
    if best is None or best.mean()<.7:raise ValueError('支撑平面不一致')
    inliers=pts[best];center=inliers.mean(0);_,_,v=np.linalg.svd(inliers-center,full_matrices=False);n=v[-1]
    if n[2]<0:n=-n
    return center,n


def analyze(frame, settings=None, now=None, *, allow_occupied=False):
    p=policy(settings);now=time.time() if now is None else now
    rgb=np.asarray(frame['rgb']);xyz=np.asarray(frame['xyz'],float);t=rigid(frame['base_from_color'])
    if (str(frame.get('camera_id')) not in ('d435','d405') or rgb.dtype!=np.uint8
        or rgb.ndim!=3 or rgb.shape[2]!=3 or xyz.shape!=rgb.shape or not np.isfinite(xyz).all()):
        raise ValueError('RGB-D相机、尺寸或深度无效')
    stamp=float(frame['image_time']);arm=float(frame['arm_time']);body=float(frame['body_time'])
    if not np.isfinite([stamp,arm,body,now]).all() or not -.1<=now-stamp<=p['max_frame_age_s'] or max(abs(stamp-arm),abs(stamp-body))>.2:
        raise ValueError('深度帧过期或与反馈不配对')
    if not str(frame.get('id','')) or not str(frame.get('calibration_sha256','')):raise ValueError('缺少快照或外参身份')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=65)).astype(np.uint8)*255
    red=cv2.morphologyEx(red,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,hierarchy=cv2.findContours(red,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE);rings=[]
    if hierarchy is not None:
        for i,c in enumerate(contours):
            child=hierarchy[0,i,2];area=cv2.contourArea(c)
            if hierarchy[0,i,3]==-1 and child>=0 and area>800:
                inner=contours[child];ratio=cv2.contourArea(inner)/area
                if .4<ratio<.99 and min(cv2.boundingRect(c)[2:])>30:rings.append((c,inner))
    if len(rings)!=1:raise ValueError('需要唯一、完整的红圈闭环')
    outer,inner=rings[0];inside=np.zeros(red.shape,np.uint8);cv2.drawContours(inside,[inner],-1,1,-1)
    valid=(xyz[:,:,2]>.07)&(xyz[:,:,2]<3.)
    if (valid&(inside>0)).sum()<.65*max(1,inside.sum()):raise ValueError('圈内有效支撑深度不足')
    world=xyz@t[:3,:3].T+t[:3,3]
    center,n=_plane(world[valid&(inside>0)],p['plane_residual_mm']/1000)
    tilt=float(np.degrees(np.arccos(np.clip(n[2],-1,1))))
    if tilt>p['max_plane_tilt_deg']:raise ValueError('支撑平面倾斜过大')
    left=np.array([0.,1.,0.]);left-=n*(left@n);left/=np.linalg.norm(left);forward=np.cross(left,n)
    contour=np.concatenate([outer[:,0,:],inner[:,0,:]])
    uv=contour; good=valid[uv[:,1],uv[:,0]]
    if good.mean()<.65:raise ValueError('红圈边界有效深度不足')
    boundary=world[uv[good,1],uv[good,0]];height=(boundary-center)@n
    if np.mean(abs(height)<.01)<.8:raise ValueError('红圈不在支撑平面')
    boundary=boundary[abs(height)<.01];coords=np.column_stack(((boundary-center)@forward,(boundary-center)@left))
    a=np.column_stack((2*coords,np.ones(len(coords))));cx,cy,k=np.linalg.lstsq(a,np.sum(coords**2,axis=1),rcond=None)[0]
    radius=float(np.sqrt(max(0,k+cx*cx+cy*cy)));residual=np.abs(np.linalg.norm(coords-[cx,cy],axis=1)-radius)
    if not .04<radius<1 or np.percentile(residual,90)>p['ring_residual_mm']/1000:raise ValueError('红圈尺寸或圆形拟合不一致')
    center=center+forward*cx+left*cy
    size=np.asarray(p['battery_size_m']);margin=p['edge_margin_mm']/1000;half=size[:2]/2
    offset=max(half[1]+p['gap_mm']/2000+.005,radius*.45)
    target=center+left*offset
    corners=np.array([[-half[0],-half[1]],[half[0],-half[1]],[half[0],half[1]],[-half[0],half[1]]])
    # Both nominal batteries must fit, including the allowed yaw envelope.
    worst=0.;envelope=half.copy()
    for angle in np.linspace(-p['yaw_tolerance_deg'],p['yaw_tolerance_deg'],17):
        c,s=np.cos(np.radians(angle)),np.sin(np.radians(angle));q=corners@np.array([[c,s],[-s,c]])
        worst=max(worst,float(np.linalg.norm(q+[0,offset],axis=1).max()))
        envelope=np.maximum(envelope,np.max(abs(q),axis=0))
    available=min(radius-margin-worst,offset-half[1]-margin)
    tolerance=min(p['xy_tolerance_mm']/1000,available/np.sqrt(2))
    if tolerance<.003:raise ValueError('红圈尺寸不足以容纳两块电池和安全空间')
    relative=world-target;u=relative@forward;v=relative@left;height=relative@n
    verified_half=envelope+tolerance
    footprint=valid&(abs(u)<verified_half[0])&(abs(v)<verified_half[1])
    if footprint.sum()<60 or (not allow_occupied and (abs(height[footprint])<=.01).mean()<.85):raise ValueError('左侧支撑不足或已占用')
    if not allow_occupied and np.count_nonzero(footprint&(height>.02))>10:raise ValueError('左侧区域已占用')
    if not allow_occupied:
        for lo_u,hi_u in zip(np.linspace(-verified_half[0],verified_half[0],5)[:-1],np.linspace(-verified_half[0],verified_half[0],5)[1:]):
            for lo_v,hi_v in zip(np.linspace(-verified_half[1],verified_half[1],5)[:-1],np.linspace(-verified_half[1],verified_half[1],5)[1:]):
                cell=valid&(u>=lo_u)&(u<hi_u)&(v>=lo_v)&(v<hi_v)&(abs(height)<.01)
                if cell.sum()<3:raise ValueError('左侧底面支撑深度覆盖不足')
    points=target+corners[:,0,None]*forward+corners[:,1,None]*left
    return dict(status='localized',side='left',left_definition='base_link +Y projected onto support',
        snapshot_id=str(frame['id']),camera=str(frame['camera_id']),image_time=stamp,
        calibration_sha256=str(frame['calibration_sha256']),center_base_m=center.tolist(),
        normal_base=n.tolist(),left_axis_base=left.tolist(),forward_axis_base=forward.tolist(),
        radius_m=radius,target_base_m=target.tolist(),footprint_base_m=points.tolist(),
        verified_half_extent_m=verified_half.tolist(),battery_size_m=size.tolist(),xy_tolerance_mm=float(tolerance*1000),
        z_tolerance_mm=p['z_tolerance_mm'],yaw_tolerance_deg=p['yaw_tolerance_deg'],
        plane_tilt_deg=tilt,plane_residual_mm=p['plane_residual_mm'],
        ring_contour_px=outer[:,0,:][::max(1,len(outer)//160)].tolist(),executable=False)
