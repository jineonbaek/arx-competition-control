"""Compare depth on both battery shoulders and the handle with an operator-seated sample."""
import hashlib
import threading
from contextlib import contextmanager
import json

import cv2
import numpy as np

from agent.docking_fit import OBJECT_TO_TOOL
from d405_left_edges import _rim_plane, battery_geometry, paired_geometry, insertion_mouth

SEAT_TOLERANCE_MM = 2.


class SeatFeaturesHidden(ValueError):
    """One of the surfaces required to release is not visible."""


def geometry_hashes(root):
    paths=('config/right-grip-tcp.json','config/battery-envelope.json','web/public/cad/battery-grip.json')
    return {name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in paths}


def seat_signature(frame,tool,root):
    rgb,xyz,transform,_,_=paired_geometry(frame)
    mask,_,valid,n,offset,normal,_=_rim_plane(rgb,xyz,transform)
    rim=mask & valid & (np.abs(xyz@n-offset)<.003)
    # Only sample points flanked by this same connected red rim, below its top rail.
    interior=(np.maximum.accumulate(rim,axis=1) & np.maximum.accumulate(rim[:,::-1],axis=1)[:,::-1]
              & np.maximum.accumulate(rim,axis=0) & ~mask)
    _,contact=battery_geometry(root)
    base=xyz@transform[:3,:3].T+transform[:3,3]
    obj=(base-tool[:3,3])@(tool[:3,:3]@OBJECT_TO_TOOL)*1000.+contact
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    dark=valid & interior & (hsv[:,:,1]<100) & (hsv[:,:,2]<175)
    outside=-(xyz@n-offset)*np.sign(normal[0])*1000.
    x,y,z=obj.transpose(2,0,1)
    shoulders=[];levels=[];counts=[]
    for side in (1,-1):
        # Battery CAD shoulder panels flanking the central handle, in object millimetres.
        region=dark & (side*y>16.) & (side*y<33.) & (np.abs(x)<32.) & (np.abs(z-154.)<10.)
        points=obj[region];dist=outside[region]
        if len(points)<50:
            raise SeatFeaturesHidden('D405 缺少电池本体一侧的肩面深度')
        keep=np.abs(dist-np.median(dist))<=2.
        cells=np.unique(np.floor(points[keep,:2]/3.),axis=0)
        if keep.sum()<50 or keep.mean()<.7 or len(cells)<8 or np.min(np.ptp(cells,axis=0))<2:
            raise SeatFeaturesHidden('D405 电池肩面有效范围不足')
        shoulders.append(float(np.median(dist[keep])))
        levels.append(float(np.median(points[keep,2])))
        counts.append(int(keep.sum()))
    # Compare the same central 4 mm section of the dark handle across exposures.
    handle=dark & (np.abs(y)<8.) & (z>181.) & (z<185.) & (np.abs(x)<40.)
    if handle.sum()<45 or np.ptp(y[handle])<9. or np.ptp(z[handle])<2.:
        raise SeatFeaturesHidden('D405 缺少夹爪之间的把手深度')
    axis=normal*np.sign(normal[0]);anchor=transform[:3,:3]@(n*offset)+transform[:3,3]
    return dict(shoulders_mm=shoulders,surface_z_mm=levels,handle_outside_mm=float(np.median(outside[handle])),
                support_pixels=counts+[int(handle.sum())],source_frame=str(frame['id']),
                calibration=str(frame['calibration_sha256']),camera_serial=json.loads(str(frame['intrinsics']))['serial'],
                mouth=dict(inside_axis_base=axis.tolist(),mouth_anchor_base_m=anchor.tolist()))


def compare_seat(standard,sample):
    ref=standard['signature']
    if (ref['source_frame']==sample['source_frame'] or ref['calibration']!=sample['calibration']
            or ref['camera_serial']!=sample['camera_serial']):
        raise ValueError('坐底标准需要同一标定下的独立新曝光')
    offsets=np.asarray(sample['shoulders_mm'])-np.asarray(ref['shoulders_mm'])
    levels=np.asarray(sample['surface_z_mm'])-np.asarray(ref['surface_z_mm'])
    handle=float(sample['handle_outside_mm'])
    handle_delta=handle-float(ref['handle_outside_mm'])
    if (offsets.shape!=(2,) or levels.shape!=(2,) or not np.isfinite(np.r_[offsets,levels,handle,handle_delta]).all()
            or np.max(np.abs(levels))>3.):
        raise ValueError('本次深度点与坐底样本的肩面不对应')
    coherent=handle>0. and np.ptp(offsets)<=3. and abs(handle_delta-np.mean(offsets))<=3.
    match=bool(coherent and np.max(np.abs(offsets))<=SEAT_TOLERANCE_MM and abs(handle_delta)<=3.)
    return dict(reference_id=standard['reference_id'],reference_match=match,handle_outside_mm=handle,
                allow_compensation=bool(coherent and np.min(offsets)>SEAT_TOLERANCE_MM),
                remaining_mm=float(np.min(offsets)),shoulder_errors_mm=offsets.tolist(),handle_error_mm=handle_delta,
                mouth=sample['mouth'])


def partial_body_range(frame,tool,root,standard):
    """Measure a partly inserted body when the seating features are occluded."""
    from mouth_push import measured_end
    rgb,xyz,transform,rs,intr=paired_geometry(frame)
    ref=standard['signature']
    if (str(frame['id'])==ref['source_frame'] or str(frame['calibration_sha256'])!=ref['calibration']
            or json.loads(str(frame['intrinsics']))['serial']!=ref['camera_serial']):
        raise ValueError('D405 补推测距缺少同一标定下的新曝光')
    mouth=insertion_mouth(rgb,xyz,transform,rs,intr)
    mask,_,valid,n,offset,normal,_=_rim_plane(rgb,xyz,transform)
    rim=mask & valid & (np.abs(xyz@n-offset)<.003)
    inside=(np.maximum.accumulate(rim,axis=1) & np.maximum.accumulate(rim[:,::-1],axis=1)[:,::-1]
            & np.maximum.accumulate(rim,axis=0) & ~mask)
    bounds,contact=battery_geometry(root)
    xyz_base=xyz@transform[:3,:3].T+transform[:3,3]
    obj=(xyz_base-tool[:3,3])@(tool[:3,:3]@OBJECT_TO_TOOL)*1000.+contact
    # The exposed flat face at CAD +X must extend behind this same rim.
    signed=(xyz@n-offset)*np.sign(normal[0])*1000.
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    panel=(inside & valid & (signed>3.) & (hsv[:,:,1]<100) & (hsv[:,:,2]<175)
           & (np.abs(obj[:,:,0]-bounds[1,0])<4.)
           & (obj[:,:,1]>=bounds[0,1]-4.) & (obj[:,:,1]<=bounds[1,1]+4.)
           & (obj[:,:,2]>=bounds[0,2]) & (obj[:,:,2]<148.))
    cells=np.unique(np.floor(obj[panel,1:]/3.),axis=0)
    if len(cells)<18 or np.min(np.ptp(cells,axis=0))<3:
        raise ValueError('D405 尚未看到足够的仓内电池面，继续观测')
    end=measured_end(frame,tool,root)
    depth=float((np.asarray(end['point_base_m'])-mouth['mouth_anchor_base_m'])@mouth['inside_axis_base'])*1000.
    remaining=standard['seated_depth_mm']-depth
    # Leave the last compensation step to the operator-taught seating image.
    limit=remaining-10.
    if not np.isfinite(depth) or depth<=0. or limit<=SEAT_TOLERANCE_MM:
        raise ValueError('D405 正在等待坐底标准中的电池肩面和把手')
    return dict(source='d405_partial_body',body_depth_mm=depth,mouth=mouth,handle_outside_mm=None,
                reference_match=False,allow_compensation=True,remaining_mm=remaining,compensation_limit_mm=limit,
                range_method=end['method'],body_support_cells=len(cells))


def measure_seated_reference(capture,controller,tcp,guard,root,frame_reader):
    from time import time
    path=root/'config/seated-reference.json'
    if not path.is_file():raise ValueError('未找到当前完全入仓标准')
    standard=json.loads(path.read_text())
    if standard['geometry_sha256']!=geometry_hashes(root):raise ValueError('坐底标准使用的夹爪或电池标定已变化')
    guard();frame=frame_reader(capture)
    stamp=float(frame['image_time'])
    tool=controller.models['right'].wrist_transform(frame['joints'],float(frame['lift_rad']))@tcp
    try:
        sample=seat_signature(frame,tool,root)
    except SeatFeaturesHidden:
        report=partial_body_range(frame,tool,root,standard)
    else:
        result=compare_seat(standard,sample)
        report=dict(result,source='d405_operator_seated_reference',signature=sample,
                    body_depth_mm=float(standard['seated_depth_mm'])-result['remaining_mm'])
    guard();state=controller.snapshot()
    current=controller.models['right'].wrist_transform(state['arms']['right']['joints'],state['body']['height'])@tcp
    if (not -.05<=time()-stamp<=4. or not state['arms']['right']['fresh'] or not state['body']['fresh']
            or np.linalg.norm(tool[:3,3]-current[:3,3])>.0015
            or np.trace(tool[:3,:3].T@current[:3,:3])<1+2*np.cos(np.radians(.5))):
        raise ValueError('D405 坐底画面与当前关节反馈不匹配')
    return dict(report,cameras={'d405':dict(image_time=stamp,frame_id=str(frame['id']))})


@contextmanager
def monitor_stroke(task,camera,model,tcp,root):
    """Display moving-frame estimates. Only the independent stationary verifier can release."""
    import time
    stop=threading.Event()
    def check():
        if stop.is_set():raise ValueError('观察已结束')
    def observe():
        try:
            standard=json.loads((root/'config/seated-reference.json').read_text())
            if standard['geometry_sha256']!=geometry_hashes(root):return
            while not stop.is_set():
                try:
                    frame=camera.live_frame(check,1.5,monitor_only=True)
                    tool=model.wrist_transform(frame['joints'],float(frame['lift_rad']))@tcp
                    result=compare_seat(standard,seat_signature(frame,tool,root))
                    live=dict(frame_id=str(frame['id']),image_time=float(frame['image_time']),
                              shoulder_errors_mm=result['shoulder_errors_mm'],remaining_mm=result['remaining_mm'],
                              handle_outside_mm=result['handle_outside_mm'],recognized=True)
                except (ValueError,RuntimeError,OSError) as error:
                    live=dict(recognized=False,reason=str(error))
                if not stop.is_set():
                    with task.lock:task.state['d405_live']=dict(live,updated_at=time.time())
                stop.wait(.05)
        except (ValueError,OSError,KeyError) as error:
            with task.lock:task.state['d405_live']=dict(recognized=False,reason=str(error),updated_at=time.time())
    thread=threading.Thread(target=observe,name='d405-moving-observation',daemon=True)
    thread.start()
    try:yield
    finally:
        stop.set();thread.join(3.)
