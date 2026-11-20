"""D405 partial-rim visual guidance; produces observations, never motor commands."""
import cv2
import numpy as np

def _runs(values):
    indexes=np.flatnonzero(values)
    return np.split(indexes,np.flatnonzero(np.diff(indexes)>1)+1) if len(indexes) else []

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
    # A battery can occlude the connecting edge. Only join a unique pair of
    # substantial, separated upper-clipped components; ambiguous scenes fail.
    if y < 2 and w < 150:
        pairs=[]
        for pos,i in enumerate(candidates):
            for j in candidates[pos+1:]:
                a,b=sorted((stats[i],stats[j]),key=lambda s:s[0])
                ax,ay,aw,ah,aa=map(int,a);bx,by,bw,bh,ba=map(int,b)
                gap=bx-(ax+aw); span=bx+bw-ax
                overlap=min(ay+ah,by+bh)-max(ay,by)
                if ay<2 and by<2 and min(aa,ba)>500 and gap>=8 and (bx+bw/2)-(ax+aw/2)>.45*span and overlap>=20 and 80<=span<=450:
                    pairs.append((a,b))
        if len(pairs)==1:
            a,b=pairs[0];x=int(a[0]);y=min(int(a[1]),int(b[1]));w=int(b[0]+b[2])-x;h=max(int(a[1]+a[3]),int(b[1]+b[3]))-y
        elif len(pairs)>1:
            raise ValueError('D405 red rim ambiguous')
    clipped_top=y<2
    if x<2 or x+w>=W-2 or y+h>=H-2 or w<50 or h<30:
        raise ValueError('D405 red rim is clipped or too small')
    rows=[]
    for yy in range(y if clipped_top else y+int(.2*h),y+int(.8*h)):
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
    region=np.zeros((H,W),bool);region[max(0,y+int(.25*h)):min(H,y+int((3.0 if clipped_top else 1.25)*h)),max(0,x-int(.4*w)):min(W,x+int(1.4*w))]=True
    ys,xs=np.nonzero(body&region)
    if not len(ys):raise ValueError('D405 battery front edge unavailable')
    front=int(ys.min());edges=[]
    front_band=max(12,int(.4*h)) if clipped_top else max(5,int(.12*h))
    for yy in range(front,min(H,front+front_band)):
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
    yy,xx=np.indices((H,W));band=(yy>=front)&(yy<front+front_band)&body&region
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
                visible_rim_rows=len(rows),visible_front_rows=len(edges),rim_clipped_top=clipped_top,executable=False,
                note='Partial-edge perspective estimate; unverified physical accuracy, requires path validation')
