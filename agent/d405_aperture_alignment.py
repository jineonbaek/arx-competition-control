"""D405 partial-rim visual guidance; produces observations, never motor commands."""
import cv2
import numpy as np

def _runs(values):
    indexes=np.flatnonzero(values)
    return np.split(indexes,np.flatnonzero(np.diff(indexes)>1)+1) if len(indexes) else []

def _scalar(value):
    value=np.asarray(value)
    return value.item() if value.shape==() else value

def estimate_horizontal_alignment(frame,*,target_offset_mm,correction_axis_base=(0.,1.,0.),
                                  max_step_mm=3.,expected_calibration_sha256,after_image_time=None):
    """Compare visible D405 rim and battery-front evidence along one base axis."""
    rgb=np.asarray(frame.get('rgb'));xyz=np.asarray(frame.get('xyz'),float);T=np.asarray(frame.get('base_from_color'),float)
    axis=np.asarray(correction_axis_base,float);camera=str(_scalar(frame.get('camera_id','')))
    calibration=str(_scalar(frame.get('calibration_sha256','')));ident=str(_scalar(frame.get('id','')))
    try:image_time=float(_scalar(frame['image_time']));arm_time=float(_scalar(frame['arm_time']));body_time=float(_scalar(frame['body_time']))
    except (KeyError,TypeError,ValueError,OverflowError) as error:raise ValueError('Invalid D405 frame timing') from error
    if (camera!='d405' or calibration!=expected_calibration_sha256 or not ident
            or not isinstance(expected_calibration_sha256,str) or len(expected_calibration_sha256)!=64):
        raise ValueError('D405 camera or calibration identity mismatch')
    if (rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8 or xyz.shape!=rgb.shape
            or T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1])
            or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)
            or not np.isfinite([image_time,arm_time,body_time,target_offset_mm,max_step_mm]).all()
            or max(abs(image_time-arm_time),abs(image_time-body_time))>.20
            or not 0<max_step_mm<=3.):
        raise ValueError('Invalid D405 horizontal alignment frame')
    length=float(np.linalg.norm(axis))
    if not np.isfinite(length) or length<.99 or length>1.01:raise ValueError('Invalid D405 correction axis')
    axis/=length
    if after_image_time is not None and (not np.isfinite(after_image_time) or image_time<=float(after_image_time)):
        raise ValueError('Correction requires a newer D405 frame')

    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)).astype(np.uint8)
    n,labels,stats,_=cv2.connectedComponentsWithStats(red,8)
    candidates=[i for i in range(1,n) if stats[i,4]>300]
    if not candidates:raise ValueError('D405 red rim unavailable')
    index=max(candidates,key=lambda i:stats[i,4]);x,y,w,h=map(int,stats[index,:4]);H,W=red.shape
    if min(x,y)<2 or x+w>=W-2 or y+h>=H-2 or w<50 or h<30:
        raise ValueError('D405 red rim is clipped or too small')
    rows=[]
    for yy in range(y+int(.2*h),y+int(.8*h)):
        runs=[a for a in _runs(red[yy,x:x+w]>0) if len(a)>=max(4,int(w*.035))]
        if len(runs)>=2 and runs[-1][0]-runs[0][-1]>=.3*w:
            rows.append((yy,int(runs[0][-1]+x),int(runs[-1][0]+x)))
    if len(rows)<8:raise ValueError('D405 visible rim side edges insufficient')
    rows=np.asarray(rows,int)

    black=(hsv[:,:,2]<100).astype(np.uint8);n,labels,stats,centers=cv2.connectedComponentsWithStats(black,8)
    candidates=[i for i in range(1,n) if stats[i,4]>.02*H*W]
    if not candidates:raise ValueError('D405 battery front edge unavailable')
    idx=min(candidates,key=lambda i:np.linalg.norm((centers[i]-[W/2,H*.45])/[W,H]));body=labels==idx
    region=np.zeros((H,W),bool);region[max(0,y+int(.25*h)):min(H,y+int(1.5*h)),max(0,x-int(.4*w)):min(W,x+int(1.4*w))]=True
    ys,xs=np.nonzero(body&region)
    if not len(ys):raise ValueError('D405 battery front edge unavailable')
    front=int(ys.min());edge_pixels=[]
    for yy in range(front,min(H,front+max(5,int(.12*h)))):
        runs=[a for a in _runs(body[yy]&region[yy]) if len(a)>=.25*(rows[:,2]-rows[:,1]).mean()]
        if runs:
            run=max(runs,key=len);edge_pixels.extend((yy,int(xx)) for xx in run)
    if len(edge_pixels)<40:raise ValueError('D405 battery front edge insufficient')

    rim_pixels=np.vstack((rows[:,[0,1]],rows[:,[0,2]]));edge_pixels=np.asarray(edge_pixels,int)
    def projected(pixels,label):
        points=xyz[pixels[:,0],pixels[:,1]]
        valid=np.isfinite(points).all(axis=1)&(points[:,2]>.06)&(points[:,2]<1.5)
        if valid.sum()<8:raise ValueError('D405 '+label+' depth insufficient')
        base=points[valid]@T[:3,:3].T+T[:3,3]
        return float(np.median(base@axis))
    rim_center=projected(rim_pixels,'rim edge');battery_center=projected(edge_pixels,'battery front')
    observed=(battery_center-rim_center)*1000.;raw=float(target_offset_mm-observed);step=float(np.clip(raw,-max_step_mm,max_step_mm))
    return dict(status='visual_guidance_only',camera='d405',frame_id=ident,image_time=image_time,
        calibration_sha256=calibration,axis_base=axis.tolist(),target_offset_mm=float(target_offset_mm),
        rim_center_axis_m=rim_center,battery_front_axis_m=battery_center,observed_offset_mm=observed,
        raw_error_mm=raw,step_mm=step,direction='positive_y' if step>0 else ('negative_y' if step<0 else 'aligned'),
        rim_inner_px=rows[:,[1,2]].astype(float).tolist(),rim_rows_px=rows[:,0].astype(float).tolist(),
        battery_front_px=edge_pixels.astype(float).tolist(),visible_rim_rows=len(rows),visible_front_pixels=len(edge_pixels),
        executable=False,note='Visible depth-supported edges in base_link; motion authorization requires route guards')

def estimate_vertical_alignment(frame,*,max_step_mm=3.):
    rgb=np.asarray(frame['rgb']);xyz=np.asarray(frame['xyz'],float);T=np.asarray(frame['base_from_color'],float)
    if (str(frame['camera_id'])!='d405' or rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8
            or xyz.shape!=rgb.shape or T.shape!=(4,4) or not np.isfinite(T).all()
            or not np.isfinite(max_step_mm) or not 0<max_step_mm<=3.):
        raise ValueError('Invalid D405 alignment frame')
    # This formulation applies only when image X represents world Z.
    if abs(T[2,0])<.85 or abs(T[2,1])>.35 or abs(T[2,2])>.35:
        raise ValueError('D405 image horizontal axis is not aligned with world height')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)).astype(np.uint8)
    n,labels,stats,_=cv2.connectedComponentsWithStats(red,8)
    candidates=[i for i in range(1,n) if stats[i,4]>300]
    if not candidates:raise ValueError('D405 red rim unavailable')
    index=max(candidates,key=lambda i:stats[i,4]);x,y,w,h=map(int,stats[index,:4]);H,W=red.shape
    if min(x,y)<2 or x+w>=W-2 or y+h>=H-2 or w<50 or h<30:
        raise ValueError('D405 red rim is clipped or too small')
    rows=[]
    for yy in range(y+int(.2*h),y+int(.8*h)):
        runs=[a for a in _runs(red[yy,x:x+w]>0) if len(a)>=max(4,int(w*.035))]
        if len(runs)>=2 and runs[-1][0]-runs[0][-1]>=.3*w:
            rows.append((yy,int(runs[0][-1]+x),int(runs[-1][0]+x)))
    if len(rows)<8:raise ValueError('D405 visible rim side edges insufficient')
    rows=np.asarray(rows,int);left,right=np.median(rows[:,1:],axis=0)
    if right-left<40:raise ValueError('D405 opening width insufficient')
    black=(hsv[:,:,2]<100).astype(np.uint8);n,labels,stats,centers=cv2.connectedComponentsWithStats(black,8)
    candidates=[i for i in range(1,n) if stats[i,4]>.02*H*W]
    if not candidates:raise ValueError('D405 held battery silhouette unavailable')
    idx=min(candidates,key=lambda i:np.linalg.norm((centers[i]-[W/2,H*.45])/[W,H]));body=labels==idx
    region=np.zeros((H,W),bool);region[max(0,y+int(.25*h)):min(H,y+int(1.25*h)),max(0,x-int(.4*w)):min(W,x+int(1.4*w))]=True
    ys,xs=np.nonzero(body&region)
    if not len(ys):raise ValueError('D405 battery front edge unavailable')
    front=int(ys.min());edges=[]
    for yy in range(front,min(H,front+max(5,int(.12*h)))):
        runs=[a for a in _runs(body[yy]&region[yy]) if len(a)>=.25*(right-left)]
        if runs:
            a=max(runs,key=len);edges.append((int(a[0]),int(a[-1])))
    if len(edges)<4:raise ValueError('D405 battery front edge insufficient')
    body_left,body_right=np.median(np.asarray(edges),axis=0)
    world_heights=[]
    for column,median in ((1,left),(2,right)):
        pixels=[]
        for yy,lo,hi in rows:
            xx=int((lo,hi)[column-1])
            if abs(xx-median)>4:continue
            point=xyz[yy,xx]
            if np.isfinite(point).all() and .06<point[2]<1.5:pixels.append(float((T[:3,:3]@point+T[:3,3])[2]))
        if len(pixels)<4:raise ValueError('D405 rim edge depth insufficient')
        world_heights.append(float(np.median(pixels)))
    physical_width_mm=abs(world_heights[1]-world_heights[0])*1000
    if not 40.<=physical_width_mm<=130.:raise ValueError('D405 measured opening height implausible')
    # Estimate the front centre at its own depth, not the rim depth.  A local
    # linear ray fit uses the frame's deprojected XYZ and also handles the
    # camera's configured distortion model without inventing coefficients.
    yy,xx=np.indices((H,W));band=(yy>=front)&(yy<front+max(5,int(.12*h)))&body&region
    good=band & np.isfinite(xyz).all(axis=2) & (xyz[:,:,2]>.06) & (xyz[:,:,2]<1.5)
    bx=xx[good].astype(float);by=yy[good].astype(float);points=xyz[good]
    if len(points)<40:raise ValueError('D405 battery front depth insufficient')
    tip_depth=float(np.median(points[:,2]));cx=float((body_left+body_right)/2.);cy=float(np.median(by))
    design=np.column_stack((bx-cx,by-cy,np.ones(len(bx))))
    rayfit,_,rank,_=np.linalg.lstsq(design,points[:,:2]/points[:,2,None],rcond=None)
    if rank<3 or not np.isfinite(rayfit).all():raise ValueError('D405 front ray fit degenerate')
    camera_point=np.r_[rayfit[0]*(cx-cx)+rayfit[1]*(cy-cy)+rayfit[2],1.]*tip_depth
    body_center_z=float((T[:3,:3]@camera_point+T[:3,3])[2])
    raw_mm=float((np.mean(world_heights)-body_center_z)*1000)
    step=float(np.clip(raw_mm,-max_step_mm,max_step_mm))
    return dict(status='visual_guidance_only',camera='d405',direction='up' if step>0 else ('down' if step<0 else 'aligned'),
                raw_error_mm=raw_mm,step_mm=step,rim_inner_px=[float(left),float(right)],
                battery_front_px=[float(body_left),float(body_right)],rim_height_mm=physical_width_mm,tip_depth_m=tip_depth,
                visible_rim_rows=len(rows),visible_front_rows=len(edges),executable=False,
                note='Partial-edge perspective estimate; unverified physical accuracy, requires path validation')
