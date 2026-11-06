"""Observed handle slots associated with independently detected battery instances."""
import cv2
import copy
import numpy as np
from battery_vision import detect_handles


def associate_holes(image,boxes):
    holes=[]
    for proposal in detect_handles(image):
        polygon=np.array(proposal['hole_box']);lo=polygon.min(axis=0);hi=polygon.max(axis=0)
        owners=[]
        for index,box in enumerate(boxes):
            x1,y1,x2,y2=box['xyxy']
            if x1<=lo[0] and y1<=lo[1] and x2>=hi[0] and y2>=hi[1]:owners.append(((x2-x1)*(y2-y1),index))
        if not owners:continue
        index=min(owners)[1]
        box=boxes[index];x1,y1,x2,y2=box['xyxy'];width=x2-x1;height=y2-y1
        if proposal['long_px']<.20*min(width,height):continue
        if not box.get('partial'):
            x,y=proposal['hole_px'];ux,uy=proposal['u_axis']
            if height>=width*1.15:
                position=(y-y1)/max(height,1)
                if abs(ux)<.60 or .33<position<.67:continue
            elif width>=height*1.15:
                position=(x-x1)/max(width,1)
                if abs(uy)<.60 or .33<position<.67:continue
        holes.append(dict(battery_index=index,center_px=proposal['hole_px'],polygon_px=proposal['hole_box'],bar_candidate_px=proposal['bar_px'],evidence='visible_image_slot',threshold_support=proposal['threshold_support'],slot_width_px=proposal['long_px'],xyz_m=None))
    # The supplied battery CAD contains one handle slot per battery, not body seams.
    selected={}
    for hole in holes:
        previous=selected.get(hole['battery_index'])
        if previous is None or (hole['threshold_support'],hole['slot_width_px'])>(previous['threshold_support'],previous['slot_width_px']):selected[hole['battery_index']]=hole
    return [selected[i] for i in sorted(selected)]



def bind_measurement(source,holes):
    if not source:return None
    result=copy.deepcopy(source);point=(source.get('detection') or {}).get('hole_px')
    matched=point is not None and any(np.linalg.norm(np.array(point)-np.array(h['center_px']))<=6 for h in holes)
    result['joint_association_valid']=bool(matched)
    if not matched:
        location=dict(result.get('location',{}));location.update(status='unconfirmed_in_joint_view',bar_surface_xyz_m=None,robot_xyz_m=None,reason='Surface measurement not associated with a visible hole inside a detected battery')
        result['location']=location
    return result


def draw_features(image,boxes,holes,battery_measurement=None):
    from battery_boxes import draw_boxes
    result=draw_boxes(image,boxes)
    for i,hole in enumerate(holes,1):
        polygon=np.round(hole['polygon_px']).astype(np.int32);center=tuple(np.round(hole['center_px']).astype(int))
        cv2.polylines(result,[polygon],True,(255,180,0),2);cv2.drawMarker(result,center,(255,180,0),cv2.MARKER_CROSS,13,2)
        cv2.putText(result,'H'+str(i),(center[0]+8,center[1]-8),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,180,0),1,cv2.LINE_AA)
    cv2.rectangle(result,(0,0),(image.shape[1],64),(18,22,26),-1)
    cv2.putText(result,f'BATTERIES {len(boxes)} | VISIBLE HOLES {len(holes)} | READ ONLY',(10,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(240,240,240),1,cv2.LINE_AA)
    location=(battery_measurement or {}).get('location',{})
    detection=(battery_measurement or {}).get('detection')
    if location.get('status')=='surface_valid' and detection:
        G=tuple(np.round(detection['bar_px']).astype(int));cv2.drawMarker(result,G,(40,220,60),cv2.MARKER_CROSS,19,2)
        cv2.putText(result,'G',(G[0]+9,G[1]+14),cv2.FONT_HERSHEY_SIMPLEX,.45,(40,220,60),1,cv2.LINE_AA)
        x,y,z=np.array(location['bar_surface_xyz_m'])*1000
        line='G surface X=%+.0f Y=%+.0f Z=%.0f mm | D435 CAMERA'%(x,y,z)
    else:line='2D image detections | no inferred 3D coordinates'
    cv2.putText(result,line,(10,39),cv2.FONT_HERSHEY_SIMPLEX,.42,(235,240,245),1,cv2.LINE_AA)
    cv2.putText(result,'Blue: observed hole | Green: body | Orange: cut by image edge',(10,56),cv2.FONT_HERSHEY_SIMPLEX,.39,(215,215,215),1,cv2.LINE_AA)
    return result
