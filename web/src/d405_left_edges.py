"""Signed base-Y clearance between the observed left inner lip and held left side."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from agent.docking_fit import OBJECT_TO_TOOL
from agent.insertion_verification import UnstableInsertionFrame
from camera_geometry import camera_intrinsics
from handeye_validation import CANDIDATE
from placement_stage_geometry import _plane


def lateral_step_mm(error_mm, observed_y_m, current_y_m, origin_y_m, limit_mm=None):
    """Translate the full observed gap into travel remaining from the current pose."""
    positions = np.array([error_mm, observed_y_m, current_y_m, origin_y_m], float)
    if not np.isfinite(positions).all():
        raise ValueError('D405 水平修正目标无效')
    target_y = observed_y_m + error_mm / 1000.
    if limit_mm is not None:
        if not np.isfinite(limit_mm) or limit_mm <= 0:
            raise ValueError('D405 水平修正目标无效')
        target_y = float(np.clip(target_y,origin_y_m-limit_mm/1000.,origin_y_m+limit_mm/1000.))
    return (target_y - current_y_m) * 1000.


def _plane_points(pixels, rs, intr, normal, offset, transform):
    rays = np.array([rs.rs2_deproject_pixel_to_point(intr, list(map(float, p)), 1.) for p in pixels])
    den = rays @ normal
    if np.any(np.abs(den) < .10):
        raise ValueError('左边沿深度视角不足')
    distances = offset / den
    if not np.all((distances > .06) & (distances < 1.5)):
        raise ValueError('左边沿深度范围无效')
    return (rays * distances[:, None]) @ transform[:3, :3].T + transform[:3, 3]


def _plane_direction(base_point, direction, T, intr):
    camera = (base_point - T[:3, 3]) @ T[:3, :3]
    delta = direction @ T[:3, :3]
    uv = np.array([intr.fx, intr.fy]) * (delta[:2] * camera[2] - camera[:2] * delta[2])
    length = np.linalg.norm(uv)
    if length < 1.:
        raise ValueError('相机视角不能区分左右')
    return uv / length


def _rim_plane(rgb, xyz, T, area=None):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    red = ((hsv[:,:,0] <= 12) | (hsv[:,:,0] >= 168)) & (hsv[:,:,1] >= 90) & (hsv[:,:,2] >= 60)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(red.astype('uint8'), 8)
    components = [i for i in range(1,count) if stats[i,4] >= 300 and min(stats[i,2:4]) >= 12]
    if area is not None:
        # Rank entire connected red components by measured pixels inside the hint.
        x,y,u,v=area
        inside=np.bincount(labels[y:v,x:u].ravel(),minlength=count)
        components=sorted((i for i in components if inside[i]>=35),key=lambda i:inside[i],reverse=True)[:1]
    if not components:
        raise ValueError('未看到仓口红边')
    mask = labels == max(components, key=lambda i:stats[i,4])
    valid = np.isfinite(xyz).all(2) & (xyz[:,:,2] > .065) & (xyz[:,:,2] < 1.5)
    n, d, _, _ = _plane(xyz[mask & valid])
    nb = T[:3,:3] @ n
    if abs(nb[0]) < .85:
        raise ValueError('红边深度平面与仓口朝向不符')
    centre = np.median(xyz[mask & valid],axis=0) @ T[:3,:3].T + T[:3,3]
    return mask,red,valid,n,d,nb,centre


def _inner_lip(xyz, T, rs, intr, plane, across, sign):
    mask,red,valid,n,d,nb,centre=plane
    along=3-across  # The two in-plane base coordinates are Y (1) and Z (2).
    outward=np.zeros(3)
    outward[across]=sign
    outward[0]=-sign*nb[across]/nb[0]
    outward_image=_plane_direction(centre,outward,T,intr)
    # Scan inward from the chosen rail. The far boundary of the
    # first local red stroke is its inner lip; no opposite stroke is required.
    yy, xx = np.nonzero(mask)
    uv = np.column_stack((xx,yy))
    lateral = uv @ outward_image
    row = np.rint(uv @ np.array([-outward_image[1],outward_image[0]])).astype(int)
    candidates = []
    for k in np.unique(row):
        ids = np.flatnonzero(row==k)
        ids = ids[np.argsort(-lateral[ids])]
        cuts = np.flatnonzero(-np.diff(lateral[ids]) > 2.)+1
        run = np.split(ids,cuts)[0]
        width = lateral[run[0]] - lateral[run[-1]]
        if not 3 <= width <= 40:
            continue
        # Depth belongs to red pixels just inside this stroke; the outermost
        # edge pixel itself can have mixed foreground/background depth.
        supporting = run[max(0,len(run)-6):max(1,len(run)-1)]
        sy,sx = yy[supporting],xx[supporting]
        if not np.any(valid[sy,sx] & (abs(xyz[sy,sx]@n-d) < .007)):
            continue
        candidates.append(uv[run[-1]])
    if len(candidates) < 18:
        raise ValueError('仓口左内沿的独立深度支持不足')
    pix = np.asarray(candidates)
    pts = _plane_points(pix,rs,intr,n,d,T)
    # Parallel rails have nearly constant coordinates across the opening.
    counts=np.array([np.sum(np.abs(pts[:,across]-v)<.003) for v in pts[:,across]])
    center=pts[np.argmax(counts),across]
    keep=np.abs(pts[:,across]-center)<.004
    pts,pix=pts[keep],pix[keep]
    if len(pts)<18 or np.ptp(pts[:,along])<.024:
        raise ValueError('仓口左内沿可见长度不足')
    # On the aperture side of the lip there must be actual recessed depth.
    # This distinguishes a lone left lip from the outside of the opposite rail.
    recessed = np.zeros(len(pix), bool)
    for offset in (4., 8., 12.):
        sample = np.rint(pix - offset*outward_image).astype(int)
        sx,sy = sample.T
        inside = (sx>=0)&(sy>=0)&(sx<mask.shape[1])&(sy<mask.shape[0])
        i=np.flatnonzero(inside);sx,sy=sx[i],sy[i]
        recessed[i] |= valid[sy,sx] & (~red[sy,sx]) & ((xyz[sy,sx]@n)-d>.020)
    if recessed.sum()<max(6, .10*len(pix)):
        raise ValueError('无法区分仓口左内沿和红框外沿')
    z0=float(np.median(pts[:,along])); k,b=np.polyfit(pts[:,along]-z0,pts[:,across],1)
    if abs(k)>.18 or np.quantile(abs(pts[:,across]-b-k*(pts[:,along]-z0)),.9)>.0018:
        raise ValueError('仓口左内沿拟合不稳定')
    return pts,pix,k,b,z0,nb


def _inner_left(rgb, xyz, T, rs, intr, area=None):
    plane=_rim_plane(rgb,xyz,T) if area is None else _rim_plane(rgb,xyz,T,area)
    return _inner_lip(xyz,T,rs,intr,plane,1,1)


def _mouth_geometry(rim, normal, method):
    axis=normal if normal[0]>0 else -normal
    up=np.array([0.,0.,1.])-axis[2]*axis
    up/=np.linalg.norm(up)
    return dict(inside_axis_base=axis.tolist(),up_base=up.tolist(),
                mouth_anchor_base_m=rim.mean(axis=0).tolist(),method=method)


def insertion_mouth(rgb, xyz, T, rs, intr):
    # Every rail lies in the same measured red plane. One depth-backed inner
    # lip establishes the mouth; the battery's lateral position plays no role.
    plane=_rim_plane(rgb,xyz,T)
    for across,sign,name in ((1,1,'left'),(1,-1,'right'),(2,1,'top'),(2,-1,'bottom')):
        try:
            rim,_,_,_,_,normal=_inner_lip(xyz,T,rs,intr,plane,across,sign)
        except ValueError:
            continue
        return _mouth_geometry(rim,normal,f'depth_supported_{name}_inner_lip')
    raise ValueError('仓口红框可见，但深度未确认入口内沿，无法确定推入轴和仓口平面')


def _left_battery(rgb, xyz, T, tool, bounds, contact, intr,area=None):
    base = xyz @ T[:3,:3].T + T[:3,3]
    rotation = tool[:3,:3] @ OBJECT_TO_TOOL
    if rotation[1,0] < .85 or abs(rotation[0,2]) < .85:
        raise ValueError('当前电池朝向不支持左侧边沿修正')
    obj = (base-tool[:3,3]) @ rotation + contact/1000.
    lo,hi=bounds/1000.
    valid=np.isfinite(xyz).all(2)&(xyz[:,:,2]>.065)&(xyz[:,:,2]<1.5)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    body=(valid & (abs(obj[:,:,0]-hi[0])<.020) &
          (obj[:,:,1]>lo[1]-.004) & (obj[:,:,1]<hi[1]+.004) &
          (obj[:,:,2]>lo[2]-.022) & (obj[:,:,2]<hi[2]-.040) &
          (hsv[:,:,2]<155) & (hsv[:,:,1]<130))
    # Recover the *measured* outward-facing side plane, using the CAD envelope
    # solely to reject the handle, jaws and background.
    if area is not None:
        x,y,u,v=area
        region=np.zeros(body.shape,bool);region[y:v,x:u]=True
        body&=region
    p=obj[body]
    if len(p)<150:
        raise ValueError('电池左侧面缺少深度点')
    rng=np.random.default_rng(42);sample=p[::max(1,len(p)//1400)];best=None
    for _ in range(100):
        a,b,c=sample[rng.choice(len(sample),3,replace=False)]
        n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if not length or abs(n[0])/length<.96:continue
        n/=length;keep=abs((sample-a)@n)<.0025
        if best is None or keep.sum()>best.sum():best=keep
    if best is None or best.sum()<80:raise ValueError('电池左侧面未确认')
    support=sample[best]
    for _ in range(2):
        mid=support.mean(0);n=np.linalg.svd(support-mid,full_matrices=False)[2][-1]
        support=p[abs((p-mid)@n)<.0025]
    if abs(n[0])<.96 or np.ptp(support[:,2])<.065 or np.ptp(support[:,1])<.029:
        raise ValueError('电池左侧面深度支持不足')
    side=body & (abs((obj-mid)@n)<.003)
    # The leading silhouette must have a real depth step toward the cabinet.
    # Invalid pixels alone cannot be interpreted as a silhouette.
    forward=-rotation[:,2]
    direction=_plane_direction(np.median(base[side],axis=0),forward,T,intr)
    near=side & (obj[:,:,2]<np.quantile(obj[side,2],.18)+.004)
    yy,xx=np.nonzero(near);H,W=near.shape
    seen=np.zeros(len(xx),bool)
    for distance in (3,6,9):
        xp=np.rint(xx+direction[0]*distance).astype(int)
        yp=np.rint(yy+direction[1]*distance).astype(int)
        inside=(xp>=0)&(xp<W)&(yp>=0)&(yp<H)
        i=np.flatnonzero(inside); xp,yp=xp[i],yp[i]
        seen[i]|=valid[yp,xp] & (xyz[yp,xp,2]-xyz[yy[i],xx[i],2]>.035)
    pix=np.column_stack((xx[seen],yy[seen]))
    if len(pix)<20:raise ValueError('电池左外沿没有可见的深度边界')
    order=np.argsort(-(pix@direction))
    scan=np.rint(pix@np.array([-direction[1],direction[0]])).astype(int)
    _,first=np.unique(scan[order],return_index=True);pix=pix[order[first]]
    pts=base[pix[:,1],pix[:,0]]
    if len(pix)<18 or np.ptp(pts[:,2])<.018:
        raise ValueError('电池左外沿可见长度不足')
    return pts,pix


def _side_at_mouth(rgb, xyz, T, tool, bounds, contact, rim, normal, rim_slope, rim_y, rim_z,area=None):
    """Estimate lateral clearance at the measured lip from the visible battery side."""
    rotation=tool[:3,:3]@OBJECT_TO_TOOL
    if rotation[1,0]<.85 or abs(rotation[0,2])<.85:
        raise ValueError('电池朝向不支持侧面测量')
    base=xyz@T[:3,:3].T+T[:3,3]
    obj=(base-tool[:3,3])@rotation*1000.+contact
    ref=rim.mean(axis=0)*1000.
    inward=normal*np.sign(normal[0])
    distance=(ref-base*1000.)@inward
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    lo,hi=bounds
    # Exclude the cabinet plane and recessed surfaces, the jaws, the handle,
    # and mixed pixels at the CAD side margins. Only observed depths vote.
    use=(np.isfinite(xyz).all(2)&(xyz[:,:,2]>.065)&(xyz[:,:,2]<1.5)
         &(distance>=8.)&(distance<=82.)
         &(obj[:,:,0]>hi[0]-30.)&(obj[:,:,0]<hi[0]+15.)
         &(obj[:,:,1]>lo[1]+4.)&(obj[:,:,1]<hi[1]-4.)
         &(obj[:,:,2]>lo[2]-22.)&(obj[:,:,2]<hi[2]-45.)
         &(hsv[:,:,2]<155)&(hsv[:,:,1]<135))
    if area is not None:
        x,y,u,v=area
        region=np.zeros(use.shape,bool);region[y:v,x:u]=True
        use&=region
    yy,xx=np.nonzero(use)
    points=base[use]*1000.;depth=distance[use]
    if len(points)<120:
        raise ValueError('仓口前缺少电池侧面深度')
    # Give each occupied 6 mm patch one vote. A close, densely sampled corner
    # must not dominate the fitted lateral face.
    tiles=np.floor(np.column_stack((depth,points[:,2]))/6.).astype(int)
    _,labels=np.unique(tiles,axis=0,return_inverse=True)
    cells=[]
    for patch in np.split(np.argsort(labels,kind='stable'),np.cumsum(np.bincount(labels))[:-1]):
        block=points[patch]
        if len(block)>=4 and np.ptp(np.percentile(block[:,1],[10,90]))<3.:
            cells.append(np.median(block,axis=0))
    if len(cells)<12:
        raise ValueError('侧面深度覆盖不足')
    cells=np.asarray(cells)
    design=np.column_stack((cells[:,0]-ref[0],cells[:,2]-ref[2],np.ones(len(cells))))
    rng=np.random.default_rng(5)
    best=np.zeros(len(cells),bool)
    for _ in range(100):
        sample=rng.choice(len(cells),3,replace=False)
        if abs(np.linalg.det(design[sample]))<30.:
            continue
        fit=np.linalg.solve(design[sample],cells[sample,1])
        if np.linalg.norm(fit[:2])>.35:
            continue
        keep=abs(design@fit-cells[:,1])<1.7
        if keep.sum()>best.sum():
            best=keep
    for _ in range(2):
        if best.sum()<max(12,.65*len(cells)):
            raise ValueError('侧面独立深度点不一致')
        fit,_,rank,_=np.linalg.lstsq(design[best],cells[best,1],rcond=None)
        if rank<3 or np.linalg.norm(fit[:2])>.35:
            raise ValueError('侧面方向无法确定')
        best=abs(design@fit-cells[:,1])<1.8
    support=cells[best]
    reach=(ref-support)@inward
    if (len(support)<max(12,.65*len(cells)) or reach.min()>22. or np.ptp(reach)<35.
            or np.ptp(support[:,2])<24. or (reach<30.).sum()<4):
        raise ValueError('侧面在仓口附近的实测覆盖不足')
    near=((depth<30.)&(abs(points[:,1]-(fit[2]+fit[0]*(points[:,0]-ref[0])+fit[1]*(points[:,2]-ref[2])))<1.8))
    if near.sum()<20:
        raise ValueError('仓口附近电池侧面不可见')
    # Compare the two faces at the same X and Z, only over their common
    # observed height. Extension to the lip is bounded by the near support.
    lower,upper=np.percentile(points[near,2],[10,90])
    lip=rim*1000.
    lip=lip[(lip[:,2]>=lower)&(lip[:,2]<=upper)]
    if len(lip)<8 or np.ptp(lip[:,2])<12.:
        raise ValueError('仓口与电池侧面缺少共同可见高度')
    z=lip[:,2]
    y=rim_y*1000.+rim_slope*(z-rim_z*1000.)
    x=ref[0]-(inward[1]*(y-ref[1])+inward[2]*(z-ref[2]))/inward[0]
    gaps=y-(fit[2]+fit[0]*(x-ref[0])+fit[1]*(z-ref[2]))
    if np.ptp(np.percentile(gaps,[10,90]))>4.:
        raise ValueError('仓口处侧面间隙分布不稳定')
    return gaps,points[near]/1000.,np.column_stack((xx[near],yy[near]))


def paired_geometry(frame):
    rgb=np.asarray(frame['rgb']);xyz=np.asarray(frame['xyz']);T=np.asarray(frame['base_from_color'])
    stamp=float(frame['image_time'])
    if (str(frame['camera_id'])!='d405' or str(frame['calibration_sha256'])!=hashlib.sha256(CANDIDATE.read_bytes()).hexdigest()
            or not np.isfinite([stamp,float(frame['arm_time']),float(frame['body_time'])]).all()
            or max(abs(stamp-float(frame[k])) for k in ('arm_time','body_time'))>.2
            or rgb.ndim!=3 or rgb.shape[-1]!=3 or xyz.shape!=rgb.shape or T.shape!=(4,4)
            or not np.isfinite(T).all() or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5)
            ):
        raise ValueError('D405 配对帧或标定无效')
    rs,intr=camera_intrinsics(json.loads(str(frame['intrinsics'])))
    return rgb,xyz,T,rs,intr


def battery_geometry(root):
    bounds=np.asarray(json.loads((Path(root)/'config/battery-envelope.json').read_text())['bounds_mm'])
    contact=np.asarray(json.loads((Path(root)/'web/public/cad/battery-grip.json').read_text())['points_object_mm']['tip'])
    return bounds, contact


def observe_left_pair(frame, tool, root, *, low_mm=0., high_mm=5., max_step_mm=None, after_image_time=None, side_at_mouth=False, regions=None):
    rgb,xyz,T,rs,intr=paired_geometry(frame)
    stamp=float(frame['image_time'])
    if (not np.isfinite([low_mm,high_mm]).all() or not low_mm<=high_mm
            or (max_step_mm is not None and (not np.isfinite(max_step_mm) or max_step_mm<=0))
            or (after_image_time is not None and stamp<=after_image_time)):
        raise ValueError('D405 左边沿测量参数或时间无效')
    if regions:
        from agent.codex_recovery import pixels
        h,w=rgb.shape[:2]
        rim,rpix,k,b,z0,normal=_inner_left(rgb,xyz,T,rs,intr,pixels(regions['rim'],(w,h),6))
        battery_area=pixels(regions['battery'],(w,h),10)
    else:
        rim,rpix,k,b,z0,normal=_inner_left(rgb,xyz,T,rs,intr)
        battery_area=None
    bounds,contact=battery_geometry(root)
    side=None
    if side_at_mouth:
        try:
            side=_side_at_mouth(rgb,xyz,T,tool,bounds,contact,rim,normal,k,b,z0,**({} if battery_area is None else {"area":battery_area}))
        except ValueError as error:
            side_reason=str(error)
    if side is not None:
        gaps,battery,bpix=side
    else:
        try:
            battery,bpix=_left_battery(rgb,xyz,T,tool,bounds,contact,intr,**({} if battery_area is None else {"area":battery_area}))
        except ValueError as error:
            if side_at_mouth:
                raise ValueError(f'{side_reason}；{error}') from error
            raise
        zlo=max(rim[:,2].min(),battery[:,2].min());zhi=min(rim[:,2].max(),battery[:,2].max())
        if zhi-zlo<.012:raise ValueError('两条左边沿缺少共同可见高度')
        overlap=(battery[:,2]>=zlo)&(battery[:,2]<=zhi)
        gaps=(b+k*(battery[overlap,2]-z0)-battery[overlap,1])*1000.
        if len(gaps)<8 or np.quantile(gaps,.9)-np.quantile(gaps,.1)>4.:
            raise UnstableInsertionFrame('两条左边沿的间隙测量不稳定', frame)
    gap=float(np.median(gaps))
    # Moving the arm in +Y reduces left clearance. Use the middle of the
    # accepted band as the target only when a correction is actually needed.
    aligned=low_mm<=gap<=high_mm
    error=0. if aligned else gap-(low_mm+high_mm)/2
    measurement=dict(status='visual_guidance_only',method='left_inner_rim_to_left_battery_edge',
                battery_reference='side_plane_at_mouth' if side is not None else 'visible_leading_edge',
                image_time=stamp,frame_id=str(frame['id']),calibration_sha256=str(frame['calibration_sha256']),
                axis_base=[0.,1.,0.],clearance_mm=gap,clearance_band_mm=[low_mm,high_mm],in_band=aligned,
                observed_y_m=float(np.asarray(tool)[1,3]),raw_error_mm=error,
                step_mm=error if max_step_mm is None else float(np.clip(error,-max_step_mm,max_step_mm)),
                rim_left_px=rpix.tolist(),battery_left_px=bpix.tolist(),executable=False)
    # All reconstructed lip points lie on the measured red face. Any one
    # anchors the depth plane; a lip point does not specify the opening height.
    mouth=_mouth_geometry(rim,normal,'depth_supported_left_inner_lip')
    return measurement,mouth,(battery,bpix)


def measure_left_edges(frame, tool, root, **options):
    options.setdefault('max_step_mm',None)
    color_only=frame.get('capture_mode')=='color_only'
    def measure(**kw):
        if color_only:
            from d405_rgb_alignment import measure_rgb_left_edges
            return measure_rgb_left_edges(frame,tool,root,**kw)
        return observe_left_pair(frame,tool,root,side_at_mouth=True,**kw)[0]
    from agent.codex_recovery import CURRENT,photos
    ctx=CURRENT.get()
    key=('d405',str(frame.get('calibration_sha256','')))
    try:
        measurement=measure(**options)
    except ValueError as error:
        if ctx is None or not ctx.frame_matches(frame):raise
        reason=str(error)
        ctx.failed('edges',key,float(frame['image_time']),lambda:dict(camera='d405',reason=reason,
                   images=photos(frame['rgb'],'edges')),threshold=1 if color_only else None)
        hint=ctx.suggestion('edges',key)
        if not hint or hint['action']!='region' or not ctx.frame_matches(frame):raise
        measurement=measure(regions=hint,**options)
        ctx.applied('edges')
        measurement['codex_region_used']=True
    if ctx:ctx.end('edges')
    return measurement
