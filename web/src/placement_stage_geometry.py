"""Measured cabinet-relative height. No robot, camera stream or command creation."""
import json
import cv2
import numpy as np
from camera_geometry import camera_intrinsics
from d405_battery_edges import fit_line,detect_face

def _runs(values):
    a=np.flatnonzero(values)
    return [x for x in np.split(a,np.flatnonzero(np.diff(a)>1)+1) if len(x)>=3] if len(a) else []

def _plane(points):
    if len(points)<120:raise ValueError('仓口有效深度不足')
    sample=points[::max(1,len(points)//1800)];rng=np.random.default_rng(41);best=None
    for _ in range(100):
        a,b,c=sample[rng.choice(len(sample),3,replace=False)]
        n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-8:continue
        n/=length;keep=abs((sample-a)@n)<.003
        if best is None or keep.sum()>best.sum():best=keep
    if best is None or best.sum()<100:raise ValueError('仓口深度平面不可用')
    selected=sample[best]
    for _ in range(3):
        center=selected.mean(0);_,_,v=np.linalg.svd(selected-center,full_matrices=False);normal=v[-1]
        if normal@center<0:normal=-normal
        distance=float(normal@center);selected=points[abs(points@normal-distance)<.003]
        if len(selected)<100:raise ValueError('仓口深度平面支持不足')
    error=float(np.quantile(abs(selected@normal-distance),.9))*1000
    if error>3.5:raise ValueError('仓口平面深度残差过大')
    return normal,distance,error,len(selected)

def cabinet_observation(frame):
    rgb=np.asarray(frame['rgb']);xyz=np.asarray(frame['xyz'],float);T=np.asarray(frame['base_from_color'],float)
    if (str(frame['camera_id'])!='d405' or rgb.ndim!=3 or rgb.dtype!=np.uint8 or xyz.shape!=rgb.shape
        or T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5)):
        raise ValueError('D405 配对帧或外参无效')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)).astype(np.uint8)
    count,labels,stats,_=cv2.connectedComponentsWithStats(red,8)
    ids=[i for i in range(1,count) if stats[i,4]>=300 and min(stats[i,2:4])>=30]
    if not ids:raise ValueError('未看到可测量的仓口红框')
    i=max(ids,key=lambda i:stats[i,4]);mask=labels==i;x,y,w,h=map(int,stats[i,:4])
    valid=np.isfinite(xyz).all(2)&(xyz[:,:,2]>.065)&(xyz[:,:,2]<1.5)
    n,d,error,support=_plane(xyz[mask&valid])
    k=json.loads(str(frame['intrinsics']));rs,intr=camera_intrinsics(k)
    def project(pixel):
        ray=np.asarray(rs.rs2_deproject_pixel_to_point(intr,list(map(float,pixel)),1.),float)
        denominator=float(ray@n)
        if abs(denominator)<.12:raise ValueError('仓口边线与视线近平行')
        point=ray*(d/denominator)
        if not .065<point[2]<1.5:raise ValueError('仓口边线深度无效')
        return T[:3,:3]@point+T[:3,3]
    pairs={}
    for name,vertical in [('left_right',True),('top_bottom',False)]:
        sides=[[],[]]
        for a in range((y if vertical else x)+int(.15*(h if vertical else w)),(y+h if vertical else x+w)-int(.15*(h if vertical else w))):
            runs=_runs(mask[a] if vertical else mask[:,a])
            if len(runs)<2 or runs[-1][0]-runs[0][-1]<.3*(w if vertical else h):continue
            for j,b in enumerate((int(runs[0][-1]),int(runs[-1][0]))):
                px,py=(b,a) if vertical else (a,b)
                if valid[py,px] and abs(float(xyz[py,px]@n)-d)<.005:sides[j].append([px,py])
        lines=[];segments=[];centers=[];directions=[]
        try:
            for side in sides:
                p=np.asarray(side,float);line,rms=fit_line(p)
                if rms>2.5:raise ValueError('内沿直线残差过大')
                axis=1 if vertical else 0;lo,hi=np.quantile(p[:,axis],[.05,.95])
                segment=np.array([[-(line[1]*v+line[2])/line[0],v] if vertical else [v,-(line[0]*v+line[2])/line[1]] for v in (lo,hi)])
                if not np.isfinite(segment).all():raise ValueError('内沿退化')
                a,b=map(project,segment);v=b-a;v/=np.linalg.norm(v)
                centers.append((a+b)/2);directions.append(v);segments.append(segment.tolist());lines.append(line.tolist())
            if abs(directions[0]@directions[1])<.97:continue
            tangent=directions[0]+directions[1]*np.sign(directions[0]@directions[1]);tangent/=np.linalg.norm(tangent)
            normal=T[:3,:3]@n;across=np.cross(normal,tangent);across/=np.linalg.norm(across)
            separation=abs(float((centers[1]-centers[0])@across))*1000
            if not 35.<separation<350.:continue
            pairs[name]=dict(anchor=np.mean(centers,axis=0),axis=across,segments=segments,
                             support=[len(v) for v in sides],separation_mm=separation)
        except (ValueError,IndexError,FloatingPointError):continue
    eligible=[(name,p) for name,p in pairs.items() if abs(p['axis'][2])>.80]
    if not eligible:raise ValueError('缺少实际相对的仓口上下内沿，不能推算离地高度')
    # The configured battery is 75 mm thick; exclude unrelated red panels and
    # signs whose measured opening height is far outside this cabinet class.
    eligible=[item for item in eligible if 55.<item[1]['separation_mm']<130.]
    if not eligible:raise ValueError('红框实测高度与电池仓尺寸不符')
    name,height=max(eligible,key=lambda item:abs(item[1]['axis'][2]));up=height['axis']*np.sign(height['axis'][2])
    lateral=None;lateral_axis=None
    for other,p in pairs.items():
        if other!=name and abs(p['axis']@up)<.12 and abs(p['axis'][1])>.70:
            lateral=p['anchor'];lateral_axis=p['axis']*np.sign(p['axis'][1])
    observation=dict(frame_id=str(frame['id']),image_time=float(frame['image_time']),
        calibration_sha256=str(frame['calibration_sha256']),height_anchor_base_m=height['anchor'].tolist(),
        up_base=up.tolist(),height_edge_pair=name,opening_height_mm=height['separation_mm'],
        lateral_anchor_base_m=None if lateral is None else lateral.tolist(),
        lateral_axis_base=None if lateral_axis is None else lateral_axis.tolist(),
        inside_axis_base=(T[:3,:3]@n).tolist(),
        plane_normal_camera=n.tolist(),plane_offset_m=d,plane_p90_mm=error,depth_support_pixels=support,
        observed_pairs={k:{key:v for key,v in p.items() if key not in ('anchor','axis')} for k,p in pairs.items()},
        method='fixed_upright_cabinet_depth_supported_inner_edges',physical_accuracy_verified=False)
    return observation

def height_reading(tool,observation,cabinet_z_mm,*,now,max_age_s=2.):
    if not 0<=now-observation['image_time']<=max_age_s:raise ValueError('D405 高度观测已过期')
    T=np.asarray(tool,float);p=np.asarray(observation['height_anchor_base_m']);up=np.asarray(observation['up_base'])
    if T.shape!=(4,4) or not np.isfinite(np.r_[T.ravel(),p,up,cabinet_z_mm]).all():raise ValueError('高度数据无效')
    relative=float(up@(T[:3,3]-p))*1000
    return dict(tcp_z_mm=float(cabinet_z_mm+relative),relative_cabinet_z_mm=relative,
                model_tcp_z_mm=float(T[2,3]*1000),visual_z_correction_mm=float(cabinet_z_mm+relative-T[2,3]*1000),
                up_base=up.tolist(),source='d405_fixed_cabinet',age_s=now-observation['image_time'])

def z_target(tool,up_base,delta_height_mm):
    T=np.asarray(tool,float).copy();up=np.asarray(up_base,float)
    if T.shape!=(4,4) or up.shape!=(3,) or not np.isfinite(np.r_[T.ravel(),up,delta_height_mm]).all() or abs(up[2])<.80:
        raise ValueError('Z 方向无效')
    T[2,3]+=float(delta_height_mm)/(1000*up[2])
    return T

def battery_front(frame,observation,*,outside_range_mm=(5.,300.),insertion_tracking=None):
    """Fit inverse depth on the observed leading face band, excluding background."""
    options={} if insertion_tracking is None else dict(insertion_tracking=insertion_tracking)
    face=detect_face(frame['rgb'],xyz=frame['xyz'],base_from_color=frame['base_from_color'],**options)
    if face is None:raise ValueError('电池四棱未确认，不能进行水平修正')
    xyz=np.asarray(frame['xyz'],float);h,w=xyz.shape[:2];yy,xx=np.indices((h,w))
    H=np.asarray(face['H_unit_face_to_image']);inverse=np.linalg.inv(H)
    pixels=np.stack((xx,yy,np.ones((h,w))),axis=-1)@inverse.T
    uv=pixels[:,:,:2]/pixels[:,:,2,None]
    T=np.asarray(frame['base_from_color']);axis=np.asarray(observation['inside_axis_base'])
    normal=T[:3,:3].T@axis
    anchor=observation['mouth_anchor_base_m'] if 'mouth_anchor_base_m' in observation else observation['height_anchor_base_m']
    distance=float(axis@(np.asarray(anchor)-T[:3,3]))
    signed=distance-xyz@normal
    keep=(uv[:,:,0]>.05)&(uv[:,:,0]<.95)&(uv[:,:,1]>=0)&(uv[:,:,1]<.15)&np.isfinite(xyz).all(2)&(xyz[:,:,2]>.07)&(signed>outside_range_mm[0]/1000)&(signed<outside_range_mm[1]/1000)
    points=xyz[keep]
    if len(points)<40:raise ValueError('电池前沿有效深度不足')
    design=np.c_[points[:,:2]/points[:,2,None],np.ones(len(points))];target=1/points[:,2]
    for _ in range(3):
        coeff,_,rank,_=np.linalg.lstsq(design,target,rcond=None)
        if rank<3:raise ValueError('电池前沿深度拟合退化')
        residual=abs(1/(design@coeff)-1/target);good=residual<.004
        if good.sum()<40:raise ValueError('电池前沿深度不一致')
        design,target=design[good],target[good]
    rs,intr=camera_intrinsics(json.loads(str(frame['intrinsics'])))
    ray=np.asarray(rs.rs2_deproject_pixel_to_point(intr,face['front_px'],1.),float)
    inv_depth=float(np.r_[ray[:2],1.]@coeff)
    if inv_depth<=0:raise ValueError('电池前沿深度无效')
    point=ray/inv_depth;T=np.asarray(frame['base_from_color'])
    return dict(point_base_m=(T[:3,:3]@point+T[:3,3]).tolist(),support_pixels=len(target),quad_px=face['quad_px'])
