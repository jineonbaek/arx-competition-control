"""Red cabinet opening + calibrated depth + constant-world-Z insertion preview.
No controller, driver, executable plan, or robot command is imported here.
"""
import time
import cv2
import numpy as np
from .placement_geometry import rigid


def _quad(contour):
    hull=cv2.convexHull(contour)
    for epsilon in (.015,.02,.025,.035,.045):
        q=cv2.approxPolyDP(hull,epsilon*cv2.arcLength(hull,True),True)[:,0,:].astype(float)
        if len(q)==4:break
    if len(q)!=4:raise ValueError('仓口内框不是完整四边形')
    center=q.mean(0);q=q[np.argsort(np.arctan2(q[:,1]-center[1],q[:,0]-center[0]))];q=np.roll(q,-int(np.argmin(q.sum(1))),axis=0)
    points=contour[:,0,:].astype(float);lines=[]
    for a,b in zip(q,np.roll(q,-1,axis=0)):
        delta=b-a;length=np.linalg.norm(delta);projection=(points-a)@delta/max(length*length,1e-9)
        distance=np.abs((delta[0]*(points[:,1]-a[1])-delta[1]*(points[:,0]-a[0])))/max(length,1e-9)
        selected=points[(projection>.15)&(projection<.85)&(distance<6)]
        if len(selected)<5:raise ValueError('仓口直边像素不足')
        vx,vy,x,y=cv2.fitLine(selected.astype(np.float32),cv2.DIST_HUBER,0,.01,.01).reshape(-1)
        lines.append((np.array([x,y]),np.array([vx,vy])))
    corners=[]
    for i in range(4):
        a,u=lines[(i-1)%4];b,v=lines[i];matrix=np.column_stack((u,-v))
        if abs(np.linalg.det(matrix))<.1:raise ValueError('仓口边缘退化')
        corners.append(a+np.linalg.solve(matrix,b-a)[0]*u)
    return np.asarray(corners)


def detect(rgb):
    rgb=np.asarray(rgb)
    if rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8:raise ValueError('无效彩色图像')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)).astype(np.uint8)*255
    red=cv2.morphologyEx(red,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,hierarchy=cv2.findContours(red,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE);full=[];partial=[]
    if hierarchy is not None:
        for i,c in enumerate(contours):
            if hierarchy[0,i,3]!=-1 or cv2.contourArea(c)<300:continue
            x,y,w,h=cv2.boundingRect(c)
            if min(x,y)<2 or x+w>=rgb.shape[1]-1 or y+h>=rgb.shape[0]-1:
                partial.append([x,y,x+w,y+h]);continue
            child=hierarchy[0,i,2]
            if child<0:continue
            inside=contours[child];ratio=cv2.contourArea(inside)/cv2.contourArea(c)
            if .35<ratio<.96 and min(cv2.boundingRect(inside)[2:])>=15:
                try:full.append((c,inside,_quad(inside)))
                except ValueError:continue
    if len(full)>1:return dict(status='ambiguous',reason='存在多个红框，未自动选择')
    if not full:return dict(status='partial' if partial else 'not_found',reason='仓口红框出界，不能补齐隐藏边' if partial else '未识别到完整红色仓口',partial_boxes_px=partial)
    outer,inner,q=full[0]
    return dict(status='complete',outer_contour_px=outer[:,0,:][::max(1,len(outer)//160)].tolist(),
        inner_contour_px=inner[:,0,:].tolist(),inner_quad_px=q.tolist(),_red=red)


def opening(profiles,mode):
    if mode not in ('demo','competition'):raise ValueError('未知电池仓模型')
    desired='demo' if mode=='demo' else 'upper_2'
    matches=[o for o in profiles[mode]['openings'] if o['slot_id']==desired]
    if len(matches)!=1:raise ValueError('模型缺少指定目标仓口')
    return matches[0]


def _ray(frame,x,y):
    xyz=frame['xyz'];h,w=xyz.shape[:2];xx,yy=np.meshgrid(np.arange(max(0,int(x)-8),min(w,int(x)+9)),np.arange(max(0,int(y)-8),min(h,int(y)+9)))
    valid=(xyz[yy,xx,2]>.07)&(xyz[yy,xx,2]<3.)
    if valid.sum()<8:raise ValueError('仓口角点周边有效深度射线不足')
    X=xx[valid];Y=yy[valid];points=xyz[Y,X];design=np.column_stack((X-x,Y-y,np.ones(len(X))))
    normalized=points[:,:2]/points[:,2,None];fit=np.linalg.lstsq(design,normalized,rcond=None)[0]
    return np.r_[fit[2],1.]


def localize(frame,profile=None,now=None):
    now=time.time() if now is None else now;stamp=float(frame['image_time']);t=rigid(frame['base_from_color'])
    if str(frame['camera_id']) not in ('d435','d405') or not -.1<=now-stamp<=2:raise ValueError('RGB-D相机或冻结帧时效无效')
    if not np.isfinite([stamp,float(frame['arm_time']),float(frame['body_time']),now]).all() or max(abs(stamp-float(frame[k])) for k in ('arm_time','body_time'))>.2:raise ValueError('图像与机器人反馈不配对')
    rgb=np.asarray(frame['rgb']);xyz=np.asarray(frame['xyz'],float)
    if xyz.shape!=rgb.shape or not np.isfinite(xyz).all():raise ValueError('配对深度尺寸或数值无效')
    detection=detect(rgb)
    if detection['status']!='complete':raise ValueError(detection['reason'])
    red=detection.pop('_red');mask=np.zeros(red.shape,np.uint8)
    cv2.fillPoly(mask,[np.array(detection['outer_contour_px'],np.int32)],1)
    cv2.fillPoly(mask,[np.array(detection['inner_contour_px'],np.int32)],0)
    valid=(mask>0)&(red>0)&(xyz[:,:,2]>.07)&(xyz[:,:,2]<3.);points=xyz[valid]
    if len(points)<80:raise ValueError('红色前沿有效深度不足，不能使用孔洞背景深度')
    points=points[::max(1,len(points)//4000)];rng=np.random.default_rng(14);best=None
    for _ in range(100):
        a,b,c=points[rng.choice(len(points),3,replace=False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-8:continue
        n/=length;inliers=abs((points-a)@n)<.005
        if best is None or inliers.sum()>best.sum():best=inliers
    if best is None or best.mean()<.65:raise ValueError('红框前沿深度不符合单一平面')
    support=points[best];origin=support.mean(0);_,_,vh=np.linalg.svd(support-origin,full_matrices=False);normal=vh[-1]
    if normal@origin>0:normal=-normal
    corners=[]
    for x,y in detection['inner_quad_px']:
        ray=_ray(frame,x,y);den=normal@ray
        if abs(den)<.05:raise ValueError('仓口与视线夹角过大')
        p=ray*((normal@origin)/den)
        if not .07<p[2]<3:raise ValueError('仓口平面交点超出配对深度范围')
        corners.append(p@t[:3,:3].T+t[:3,3])
    corners=np.asarray(corners);outward=t[:3,:3]@normal;inside=-outward
    left=(corners[0]-corners[1]+corners[3]-corners[2]);left-=outward*(left@outward);left/=np.linalg.norm(left)
    expected=np.cross([0,0,1],inside)
    if left@expected<0:left=-left
    up=np.cross(left,outward);up/=np.linalg.norm(up)
    if up[2]<.7 or np.linalg.norm(inside[:2])<.7:raise ValueError('仓口不是适合水平入仓的立面')
    width=float((np.linalg.norm(corners[0]-corners[1])+np.linalg.norm(corners[2]-corners[3]))/2*1000)
    height=float((np.linalg.norm(corners[0]-corners[3])+np.linalg.norm(corners[1]-corners[2]))/2*1000)
    if profile and (abs(width-profile['size_mm'][0])>max(8.,profile['size_mm'][0]*.08) or abs(height-profile['size_mm'][1])>max(6.,profile['size_mm'][1]*.08)):
        raise ValueError('红框实测尺寸与指定仓口模型不符')
    residual=abs((support-origin)@normal)*1000
    return {**detection,'status':'localized','snapshot_id':str(frame['id']),'camera':str(frame['camera_id']),
        'image_time':stamp,'calibration_sha256':str(frame['calibration_sha256']),
        'center_base_m':corners.mean(0).tolist(),'corners_base_m':corners.tolist(),
        'outward_normal_base':outward.tolist(),'inside_axis_base':inside.tolist(),
        'left_axis_base':left.tolist(),'up_axis_base':up.tolist(),
        'measured_size_mm':[width,height],'rim_depth_inliers':len(support),
        'rim_plane_p90_mm':float(np.percentile(residual,90)),'executable':False}


def horizontal_plan(window,slot,battery,*,tip_point_object_mm,preclearance_mm=50.,back_margin_mm=10.):
    center=np.asarray(window['center_base_m'],float);out= np.asarray(window['outward_normal_base'],float);inside=-out
    axis=inside.copy();axis[2]=0.;axis/=np.linalg.norm(axis)
    L=np.asarray(window['left_axis_base'],float);V=np.asarray(window['up_axis_base'],float)
    tool_left=L-axis*(L@axis);tool_left/=np.linalg.norm(tool_left);tool_up=np.cross(axis,tool_left)
    bounds=np.asarray(battery['bounds_mm']);length=float(bounds[1,2]-bounds[0,2]);width,height=slot['size_mm']
    measured=np.asarray(window.get('measured_size_mm',[width,height]),float)
    if measured.shape!=(2,) or not np.isfinite(measured).all() or np.any(measured<=0):raise ValueError('实测仓口尺寸无效')
    effective=np.minimum(measured,[width,height]);scale=effective/np.array([width,height]);width,height=effective
    contact=np.asarray(tip_point_object_mm,float)
    if contact.shape!=(3,) or not np.isfinite(contact).all():raise ValueError('需要完整工具接触参考点')
    lateral=contact[0]-bounds[:,0].mean();vertical=contact[1]-bounds[:,1].mean();tip_offset_mm=contact[2]-bounds[0,2]
    tcp_cross_offset=tool_left*lateral-tool_up*vertical
    depth=slot.get('first_inner_obstacle_depth_mm')
    if depth is None or not np.isfinite(depth):raise ValueError('尚未确认CAD仓内第一处障碍深度')
    if not 0<preclearance_mm<=100 or not 0<back_margin_mm<depth:raise ValueError('仓外/后壁余量无效')
    usable=depth-back_margin_mm
    cross=np.asarray(battery['cross_section_xy_mm'],float)-bounds.mean(0)[:2];cross[:,1]*=-1
    offsets=cross[:,0,None]*tool_left+cross[:,1,None]*tool_up
    normal_extent=float(np.max(offsets@inside))
    denominator=axis@inside-.5*(axis@V)/V[2]*inside[2]
    if denominator<.7:raise ValueError('仓口倾斜过大，不能保持Z水平放入')
    travel=(usable-normal_extent)/denominator
    if travel<=0:raise ValueError('仓内深度不足')
    # Balance the vertical drift of a horizontal path through a slightly tilted measured chamber.
    bias=-.5*travel*(axis@V)/V[2]
    front=center+L*(width/4/1000)+np.array([0,0,bias/1000])
    end_leading=front+axis*(travel/1000)
    cross=np.asarray(battery['cross_section_xy_mm'],float)-bounds.mean(0)[:2];cross[:,1]*=-1
    outline=np.asarray(slot['outline_mm'],float)-slot['center_mm'];outline[:,0]*=-1;outline*=scale
    minimum=float('inf');samples=[]
    for x,y in cross:
        offset=tool_left*x/1000+tool_up*y/1000
        at_front=front+offset-axis*((offset@inside+(front-center)@inside)/(axis@inside))
        at_end=end_leading+offset
        for point in (at_front,at_end):
            uv=np.array([(point-center)@L,(point-center)@V])*1000
            distance=cv2.pointPolygonTest(outline.astype(np.float32),tuple(uv),True)
            minimum=min(minimum,float(distance));samples.append(uv.tolist())
    if minimum<=0:raise ValueError('保持Z不变时电池包络会碰仓口上下或左右边缘')
    pre=front-axis*((preclearance_mm+tip_offset_mm)/1000)+tcp_cross_offset/1000
    end=end_leading-axis*(tip_offset_mm/1000)+tcp_cross_offset/1000;z=float(pre[2]*1000)
    stroke=float(np.linalg.norm(end-pre)*1000);points=np.linspace(pre*1000,end*1000,max(2,int(np.ceil(stroke/2))+1));points[:,2]=z
    return dict(status='geometric_preview',frame='base_link',unit='mm',target_slot=slot['slot_id'],side='left',
        tcp_points_base_mm=points.tolist(),tcp_pre_base_mm=points[0].tolist(),tcp_end_base_mm=points[-1].tolist(),
        leading_points_base_mm=[((front-axis*preclearance_mm/1000)*1000).tolist(),(end_leading*1000).tolist()],
        fixed_z_mm=z,z_range_mm=float(np.ptp(points[:,2])),insertion_axis_base=axis.tolist(),
        R_base_from_tool=np.column_stack((axis,tool_left,tool_up)).tolist(),
        stroke_mm=stroke,preclearance_mm=preclearance_mm,leading_insertion_depth_mm=travel,
        cad_first_obstacle_depth_mm=depth,back_margin_mm=back_margin_mm,
        battery_protrusion_mm=max(0,length-travel),nominal_cad_clearance_mm=minimum,
        window_model_size_mm=slot['size_mm'],effective_opening_size_mm=effective.tolist(),tool_contact_object_mm=contact.tolist(),
        tool_cross_offset_mm=[float(lateral),float(vertical)],aperture_envelope_samples_mm=samples,
        executable=False,whole_robot_collision_checked=False,
        blockers=['仅规划预览，未许可执行','净间隙小于常规抓取容差，外参/持物关系/精度待现场核验','夹爪、两臂、桌面与环境的完整碰撞检查尚未完成'])
