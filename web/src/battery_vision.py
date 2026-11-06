"""Experimental white battery handle geometry detector and read-only RGB-D measurement."""
import cv2
import numpy as np

HOLE_WIDTH_MM=40.0
HOLE_HEIGHT_MM=14.0
BAR_OFFSET_MM=186.01271057128906-169.74542236328125

def frames_usable(color_ms,depth_ms,depth_scale,age_s=0.0):
    return bool(np.isfinite([color_ms,depth_ms,depth_scale,age_s]).all()
                and abs(color_ms-depth_ms)<=40 and 0<depth_scale<2 and 0<=age_s<=.35)

def detect_handles(image):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);height,width=image.shape[:2];groups=[]
    # Determine body direction from a lower brightness mask. High thresholds can
    # split off the handle from a slightly dimmer body and reverse its direction.
    body_mask=cv2.inRange(hsv,np.array([0,0,190]),np.array([179,90,255]))
    bodies=cv2.findContours(body_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
    bodies=[c for c in bodies if cv2.contourArea(c)>=1500]
    for threshold in [230,240,245,250]:
        mask=cv2.inRange(hsv,np.array([0,0,threshold]),np.array([179,85,255]))
        contours,hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:continue
        for i,c in enumerate(contours):
            parent=int(hierarchy[0,i,3])
            if parent<0 or cv2.contourArea(contours[parent])<1500:continue
            area=cv2.contourArea(c);rect=cv2.minAreaRect(c);box=cv2.boxPoints(rect)
            edges=np.roll(box,-1,axis=0)-box;lengths=np.linalg.norm(edges,axis=1);j=int(np.argmax(lengths));long=float(lengths[j]);short=float(lengths[(j+1)%4])
            if not 8<=long<=130 or not 3<=short<=45 or not 1.6<=long/max(short,.001)<=7 or area<18 or area/max(long*short,1)<.35:continue
            if np.min(box[:,0])<3 or np.max(box[:,0])>width-3 or np.min(box[:,1])<3 or np.max(box[:,1])>height-3:continue
            center=np.array(rect[0]);u=edges[j]/long;v=np.array([-u[1],u[0]])
            enclosing=[c for c in bodies if cv2.pointPolygonTest(c,(float(center[0]),float(center[1])),False)>=0]
            if not enclosing:continue
            body_contour=min(enclosing,key=cv2.contourArea)
            moment=cv2.moments(body_contour);body=np.array([moment['m10']/max(moment['m00'],1),moment['m01']/max(moment['m00'],1)])
            if np.dot(center-body,v)<0:v=-v
            if abs(np.dot(center-body,v))<short:continue
            candidate={'hole_px':center.tolist(),'bar_px':(center+v*short*BAR_OFFSET_MM/HOLE_HEIGHT_MM).tolist(),'u_axis':u.tolist(),'away_axis':v.tolist(),'long_px':long,'short_px':short,'hole_box':box.tolist(),'threshold':threshold}
            group=next((g for g in groups if np.linalg.norm(center-np.array(g[0]['hole_px']))<9),None)
            if group is None:groups.append([candidate])
            else:group.append(candidate)
    supported=[g for g in groups if len({r['threshold'] for r in g})>=2]
    if not supported:return []
    results=[]
    for group in sorted(supported,key=lambda g:np.median([r['hole_px'][0] for r in g])):
        result=dict(max(group,key=lambda r:r['threshold']))
        result.update(candidate_count=len(supported),threshold_support=len({r['threshold'] for r in group}),detector='white_plastic_handle_slot_geometry',selection='leftmost_supported_slot',not_a_trained_semantic_classifier=True)
        results.append(result)
    return results

def detect_handle(image):
    results=detect_handles(image)
    return results[0] if results else None


def locate_handle(detection,image,depth_m,intr):
    result={'status':'depth_invalid','method':'solid_bar_roi_robust_depth','coordinate_frame':'d435_color_optical_frame','units':'m','bar_surface_xyz_m':None,'hole_xyz_m':None,'cad_grip_center_xyz_m':None,'robot_xyz_m':None,'valid_points':0,'reason':'No valid solid handle depth'}
    if detection is None:
        result.update(status='not_detected',reason='No supported left handle');return result
    if depth_m.shape!=image.shape[:2]:result['reason']='RGB/depth shape mismatch';return result
    center=np.array(detection['bar_px']);u=np.array(detection['u_axis']);v=np.array(detection['away_axis']);a=max(4,detection['long_px']*.32);b=max(1.5,detection['short_px']*.20)
    box=np.array([center-u*a-v*b,center+u*a-v*b,center+u*a+v*b,center-u*a+v*b]);mask=np.zeros(depth_m.shape,np.uint8);cv2.fillPoly(mask,[np.round(box).astype(np.int32)],1)
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);mask=mask.astype(bool)&(hsv[:,:,1]<90)&(hsv[:,:,2]>170)
    valid=mask&np.isfinite(depth_m)&(depth_m>.15)&(depth_m<3.)
    z=depth_m[valid];result['valid_points']=int(len(z));result['sample_box_px']=box.tolist()
    if len(z)<18 or len(z)<.35*max(1,int(mask.sum())):return result
    median=float(np.median(z));mad=float(np.median(np.abs(z-median)));valid&=np.abs(depth_m-median)<max(.008,3*mad)
    yy,xx=np.where(valid);z=depth_m[valid]
    if len(z)<18:result['reason']='Too few consistent solid depth samples';return result
    pixels=np.column_stack([xx,yy]).astype(float)
    if 'ray_fn' in intr:rays=np.asarray(intr['ray_fn'](pixels),float)
    else:rays=np.column_stack([(pixels[:,0]-intr['cx'])/intr['fx'],(pixels[:,1]-intr['cy'])/intr['fy'],np.ones(len(pixels))])
    xyz=rays*z[:,None];point=np.median(xyz,axis=0)
    result.update(status='surface_valid',bar_surface_xyz_m=point.tolist(),valid_points=len(z),depth_median_m=float(np.median(z)),depth_mad_m=float(np.median(np.abs(z-np.median(z)))),reason='',surface_is_not_cad_mid_thickness_center=True)
    # Hole pixels see the background. No hole-center depth or robot target is inferred.
    return result

def draw_result(image,detection,location):
    output=image.copy();h,w=output.shape[:2]
    if detection:
        cv2.polylines(output,[np.round(detection['hole_box']).astype(np.int32)],True,(255,180,0),2)
        H=tuple(np.round(detection['hole_px']).astype(int));G=tuple(np.round(detection['bar_px']).astype(int))
        cv2.drawMarker(output,H,(255,180,0),cv2.MARKER_CROSS,13,2)
        color=(40,220,60) if location['status']=='surface_valid' else (0,180,255)
        cv2.drawMarker(output,G,color,cv2.MARKER_CROSS,19,2)
        cv2.putText(output,'H',(H[0]+8,H[1]-6),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,180,0),1,cv2.LINE_AA)
        cv2.putText(output,'G',(G[0]+9,G[1]+14),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1,cv2.LINE_AA)
    lines=['LEFT BATTERY HANDLE | CAMERA FRAME | READ ONLY']
    if location['status']=='surface_valid':
        x,y,z=np.array(location['bar_surface_xyz_m'])*1000
        lines+=['G surface: X=%+.0f Y=%+.0f Z=%.0f mm'%(x,y,z),'Solid bar depth; H is a hole, not a measured surface']
    elif detection:lines+=['Handle candidate found; depth unavailable']
    else:lines+=['No supported left handle detected']
    cv2.rectangle(output,(0,0),(w,20+19*len(lines)),(18,22,26),-1)
    for i,line in enumerate(lines):cv2.putText(output,line,(10,20+i*19),cv2.FONT_HERSHEY_SIMPLEX,.43,(235,240,245),1,cv2.LINE_AA)
    return output
