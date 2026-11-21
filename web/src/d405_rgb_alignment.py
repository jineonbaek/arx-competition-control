"""RGB-only lateral alignment. Metric values are image/CAD estimates, not depth."""
import cv2
from d405_horizontal_recovery import FeatureUnavailable
import numpy as np
from d405_battery_edges import fit_line,intersection,projective_geometry


def color_battery_face(rgb,region=None):
    """Refit all four current RGB edges near the attached-camera battery region.

    The old approved quadrilateral seeds the search only. No seed, old contour,
    depth mask, or inferred hidden line is returned as an observation.
    """
    h,w=rgb.shape[:2]
    proposal=np.array([[276.6454,80.0009],[402.8131,80.0932],
                       [485.8121,180.8551],[209.0050,182.6607]])*[w/640.,h/480.]
    if region is not None:
        x,y,u,roi_bottom=np.asarray(region,float)*np.array([w,h,w,h])/1000.
        old_min=proposal.min(axis=0);old_span=np.ptp(proposal,axis=0)
        proposal=(proposal-old_min)/old_span*np.array([u-x,roi_bottom-y])+[x,y]
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
    enhanced=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(gray)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    red=((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)
    edge=cv2.Canny(enhanced,20,55)>0
    yy,xx=np.indices(edge.shape)
    if region is None:edge&=(xx>w*.25)&(xx<w*.83)&(yy>h*.12)&(yy<h*.44)&~red
    else:edge&=(xx>=x-15)&(xx<=u+15)&(yy>=y-15)&(yy<=roi_bottom+15)&~red
    ey,ex=np.nonzero(edge);pixels=np.column_stack((ex,ey)).astype(float)
    gx=cv2.Sobel(enhanced,cv2.CV_32F,1,0,ksize=3);gy=cv2.Sobel(enhanced,cv2.CV_32F,0,1,ksize=3)
    gradient=np.column_stack((gx[ey,ex],gy[ey,ex]));gn=np.linalg.norm(gradient,axis=1)
    rng=np.random.default_rng(24);lines=[];errors=[];supports=[]
    for index,(a,b) in enumerate(zip(proposal,np.roll(proposal,-1,axis=0))):
        direction=b-a;length=np.linalg.norm(direction);direction/=length
        normal=np.array([-direction[1],direction[0]])
        relative=pixels-a;along=relative@direction;distance=relative@normal
        keep=(along>-.08*length)&(along<1.08*length)&(abs(distance)<(24. if index in (0,2) else 14.)*w/640.)&(abs(gradient@normal)>.65*np.maximum(gn,1))
        if index==2:keep&=gy[ey,ex]<0
        selected=pixels[keep]
        if len(selected)<20:raise FeatureUnavailable('电池彩色棱线不可见，等待下一帧')
        best=None
        for _ in range(180):
            pair=selected[rng.choice(len(selected),2,replace=False)];v=pair[1]-pair[0];norm=np.linalg.norm(v)
            if norm<15:continue
            v/=norm
            if abs(v@direction)<np.cos(np.radians(16)):continue
            if index in (0,2) and abs(v[1])>np.sin(np.radians(6)):continue
            line=np.array([-v[1],v[0],v[1]*pair[0,0]-v[0]*pair[0,1]])
            good=abs(np.c_[selected,np.ones(len(selected))]@line)<1.6
            if good.sum()<18 or np.ptp(selected[good]@direction)<length*.55:continue
            score=float(good.sum())-.3*float(np.median(abs((selected[good]-a)@normal)))
            if best is None or score>best[0]:best=(score,good)
        if best is None:raise FeatureUnavailable('电池彩色棱线支持不足，等待下一帧')
        line,error=fit_line(selected[best[1]])
        if error>1.8:raise FeatureUnavailable('电池彩色棱线拟合不稳定')
        lines.append(line);errors.append(error);supports.append(int(best[1].sum()))
    q=np.array([intersection(lines[3],lines[0]),intersection(lines[0],lines[1]),
                intersection(lines[1],lines[2]),intersection(lines[2],lines[3])])
    geometry=projective_geometry(q)
    ratio=np.linalg.norm(q[1]-q[0])/np.linalg.norm(q[2]-q[3])
    if (np.max(np.linalg.norm(q-proposal,axis=1))>45.*w/640. or not .25<ratio<.9
            or q[:,1].max()>=(h*.44 if region is None else min(h,roi_bottom+20))):raise FeatureUnavailable('电池轮廓不在夹爪前的可见表面')
    return dict(quad_px=q.tolist(),edge_rms_px=errors,edge_support_pixels=supports,
                method='current_rgb_four_lines',depth_used=False,handle_excluded=True,**geometry)


def left_inner_rail(rgb,face,region=None):
    """Right boundary of the left red stroke, only where it is actually visible."""
    h,w=rgb.shape[:2];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=35)).astype(np.uint8)
    if region is not None:
        from agent.codex_recovery import pixels
        x,y,u,v=pixels(region,(w,h),padding=4)
        region_pixels=(x,y,u,v)
    # Link small paint gaps for component selection only; fit original red
    # pixels below, never the morphologically filled pixels.
    connected=cv2.morphologyEx(red,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    n,labels,stats,_=cv2.connectedComponentsWithStats(connected,8)
    q=np.asarray(face['quad_px']);front=q[0];candidates=[]
    for i in range(1,n):
        x,y,bw,bh,count=stats[i]
        if count<120 or bh<18 or abs(x-front[0])>.30*w:continue
        # Lower brightness is useful for finding a dark frame, but can expand
        # its boundary into red reflections. Prefer the bright paint core when
        # it supplies enough exposed rows; use dark pixels only when necessary.
        rows=slice(max(y,int(front[1])-35),min(y+bh,int(front[1])-3))
        core=(labels==i)&(red>0)&(hsv[:,:,2]>=60)
        fit_mask=core if np.count_nonzero(np.sum(core[rows],axis=1)>=3)>=12 else ((labels==i)&(red>0))
        points=[]
        # Below the leading edge, red pixels may end at the occluding battery,
        # rather than at the cabinet lip. Use the exposed rail above the battery.
        for row in range(max(y,int(front[1])-35),min(y+bh,int(front[1])-3)):
            xs=np.flatnonzero(fit_mask[row])
            runs=np.split(xs,np.flatnonzero(np.diff(xs)>1)+1) if len(xs) else []
            runs=[r for r in runs if 3<=len(r)<=40 and r[0]<x+max(24,.10*bw)]
            # A highlight can split this same left stroke into small runs.
            # Its aperture-facing boundary is the innermost local run, not
            # the first bright break inside the paint.
            if runs:points.append([runs[-1][-1],row])
        if len(points)<10:continue
        points=np.asarray(points,float)
        if region is not None:
            rx,ry,ru,rv=region_pixels
            points=points[(points[:,0]>=rx)&(points[:,0]<ru)&(points[:,1]>=ry)&(points[:,1]<rv)]
            if len(points)<10:continue
        # The upper end can be the rounded corner of the painted rectangle.
        # Fit the longest straight suffix nearest the battery edge, rather than
        # rejecting the true rail because its upper corner is curved.
        fitted=None
        for start in range(max(0,len(points)-11)):
            visible=points[start:]
            try:line,rms=fit_line(visible)
            except ValueError:continue
            if abs(line[0])>=.65 and rms<=2.5:
                fitted=(line,visible);break
        if fitted is None:continue
        line,points=fitted
        good=abs(np.c_[points,np.ones(len(points))]@line)<3.
        points=points[good]
        if len(points)>=12 and np.ptp(points[:,1])>=10:
            # A short unrelated red mark can have more sampled rows than the
            # real rail. Rank proximity and visible frame structure instead
            # of selecting solely by row count. No top/bottom edge is required.
            lip_x=-(line[1]*front[1]+line[2])/line[0]
            spans=[]
            for row in range(max(0,y),min(h,int(front[1])-3)):
                xs=np.flatnonzero((labels[row]==i)&(red[row]>0))
                if len(xs)>=8:spans.append(float(xs[-1]-xs[0]))
            width=max(1.,float(np.linalg.norm(q[1]-q[0])))
            frame_support=min(1.,max(spans,default=0.)/(.75*width))
            score=80.*frame_support-abs(lip_x-front[0])+.2*len(points)
            candidates.append((score,points))
    if not candidates:raise FeatureUnavailable('仓口左内沿彩色线不可见，等待下一帧')
    return max(candidates,key=lambda v:v[0])[1]


def measure_rgb_left_edges(frame,tool,root,*,low_mm=0.,high_mm=5.,max_step_mm=None,after_image_time=None,regions=None):
    import hashlib
    from d405_left_edges import CANDIDATE,battery_geometry
    from agent.docking_fit import OBJECT_TO_TOOL
    rgb=np.asarray(frame['rgb']);stamp=float(frame['image_time']);tool=np.asarray(tool,float)
    if (rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8 or frame['camera_id']!='d405'
            or str(frame['calibration_sha256'])!=hashlib.sha256(CANDIDATE.read_bytes()).hexdigest()
            or not np.isfinite([stamp,float(frame['arm_time']),float(frame['body_time']),low_mm,high_mm]).all()
            or max(abs(stamp-float(frame[k])) for k in ('arm_time','body_time'))>.2
            or not low_mm<=high_mm or (after_image_time is not None and stamp<=after_image_time)
            or (max_step_mm is not None and (not np.isfinite(max_step_mm) or max_step_mm<=0))):
        raise ValueError('D405 彩色配对帧或测量参数无效')
    if tool.shape!=(4,4) or not np.isfinite(tool).all() or (tool[:3,:3]@OBJECT_TO_TOOL)[1,0]<.85:
        raise ValueError('电池朝向不支持基座 Y 彩色修正')
    try:
        face=color_battery_face(rgb,region=regions['battery'] if regions else None);rim=left_inner_rail(rgb,face,region=regions['rim'] if regions else None)
    except ValueError as error:
        raise FeatureUnavailable(str(error)) from error
    H=np.asarray(face['H_unit_face_to_image']);q=np.asarray(face['quad_px'])
    line,_=fit_line(rim)
    # Extend the *observed* short rail only to the nearby leading RGB edge.
    # Keep the real supporting pixels in the overlay; mark the metric as an
    # image estimate rather than synthesizing a hidden observed cabinet edge.
    row=float(q[0,1]);extension=max(0.,row-float(rim[:,1].max()))
    if extension>18.*rgb.shape[0]/480.:raise FeatureUnavailable('左内沿遮挡过多，无法比较电池前棱')
    lip=np.array([-(line[1]*row+line[2])/line[0],row])
    uv=cv2.perspectiveTransform(lip[None,None].astype(np.float64),np.linalg.inv(H))[0]
    bounds,_=battery_geometry(root);width=float(bounds[1,0]-bounds[0,0])
    gap=float(-uv[0,0]*width)
    # A subpixel boundary difference is not a command to move to the centre
    # of the band. Propagate one image pixel through the same homography,
    # capped at 1 mm; preserve the nominal 0..5 mm target in the receipt.
    adjacent=np.array([[lip[0]-1.,row],[lip[0]+1.,row]],float)
    adjacent_uv=cv2.perspectiveTransform(adjacent[None],np.linalg.inv(H))[0]
    uncertainty=min(1.,float(np.max(np.abs(adjacent_uv[:,0]-uv[0,0]))*width))
    aligned=low_mm-uncertainty<=gap<=high_mm+uncertainty
    error=0. if aligned else gap-(low_mm+high_mm)/2
    edge=np.linspace(q[0],q[3],24)
    frame['battery_face_observation']=face
    return dict(status='visual_guidance_only',method='left_inner_rim_to_left_battery_edge',
        measurement_source='rgb_only',depth_used=False,metric_source='nominal_battery_width_projective_estimate',
        battery_reference='rgb_visible_edge',image_time=stamp,frame_id=str(frame['id']),
        calibration_sha256=str(frame['calibration_sha256']),axis_base=[0.,1.,0.],
        clearance_mm=gap,clearance_band_mm=[low_mm,high_mm],clearance_uncertainty_mm=uncertainty,
        alignment_pixel_tolerance=1.,in_band=aligned,
        observed_y_m=float(tool[1,3]),raw_error_mm=error,
        step_mm=error if max_step_mm is None else float(np.clip(error,-max_step_mm,max_step_mm)),
        rim_left_px=rim.tolist(),battery_left_px=edge.tolist(),
        nominal_width_mm=width,pixel_gap=float(q[0,0]-lip[0]),rim_extension_px=extension,
        comparison_lip_px=lip.tolist(),comparison_battery_px=q[0].tolist(),executable=False)
