"""Measure battery body depth and seat the body with the handle outside."""
import hashlib
import json
import time
from functools import lru_cache
from contextlib import nullcontext
from pathlib import Path

import cv2
import numpy as np

from agent.docking_fit import OBJECT_TO_TOOL
from agent.held_insertion_path import ENTRY_ANGLE_DEG
from camera_geometry import camera_intrinsics
from d405_left_edges import _rim_plane, battery_geometry, insertion_mouth, paired_geometry
from grasp_preview import HEAD_CALIBRATION

from placement_stage_geometry import battery_front
from insertion_contact import InsertionContact

ROOT = Path(__file__).resolve().parents[2]
PUSH_MM = 150.
ARRIVAL_MM = 1.5
MAX_RAIL_TRAVEL_MM = 300.
TRAVEL_EPS_MM = 1e-9
CONTACT_SEAT_ZONE_MM = 10.


@lru_cache(maxsize=2)
def end_surfaces(root):
    mesh = (root/'web/cad/gripper-v2/battery-print.stl').read_bytes()
    envelope = json.loads((root/'config/battery-envelope.json').read_text())
    if hashlib.sha256(mesh).hexdigest() != envelope['source_sha256']:
        raise ValueError('电池模型与尺寸标定不匹配')
    triangles = np.frombuffer(mesh[84:], dtype=[('n','<f4',3),('v','<f4',(3,3)),('attribute','<u2')])
    bounds,contact=battery_geometry(root)
    origin=np.floor(bounds[0,:2])-8
    size=np.ceil((bounds[1,:2]-origin+8)*2).astype(int)
    result=[]
    for side,sign in ((0,-1),(1,1)):
        z=bounds[side,2]
        faces=triangles['v'][(triangles['n'][:,2]*sign>.995)&(np.abs(triangles['v'][:,:,2]-z).max(1)<3.)]
        if not len(faces):
            raise ValueError('电池模型缺少端面')
        # Select the largest parallel planar patch near this physical end.
        levels=np.round(faces[:,:,2].mean(1),2)
        areas=np.linalg.norm(np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0]),axis=1)
        level=max(np.unique(levels),key=lambda v:areas[levels==v].sum())
        mask=np.zeros(tuple(size[::-1]),np.uint8)
        for tri in faces[levels==level]:
            cv2.fillConvexPoly(mask,np.rint((tri[:,:2]-origin)*2).astype(np.int32),1)
        result.append((sign,float(level),mask))
    return bounds,contact,origin,result


def measured_end(frame, tool, root):
    bounds,contact,origin,surfaces=end_surfaces(root)
    T=np.asarray(frame['base_from_color']);xyz=np.asarray(frame['xyz']);rgb=np.asarray(frame['rgb'])
    rotation=tool[:3,:3]@OBJECT_TO_TOOL
    camera_obj=(T[:3,3]-tool[:3,3])@rotation*1000+contact
    valid=np.isfinite(xyz).all(2)&(xyz[:,:,2]>.065)&(xyz[:,:,2]<3.)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    valid &= (hsv[:,:,1]<100)&(hsv[:,:,2]<165)
    iy,ix=np.nonzero(valid)
    obj=((xyz[iy,ix]@T[:3,:3].T+T[:3,3])-tool[:3,3])@rotation*1000+contact
    for sign,level,mask in surfaces:
        if (camera_obj[2]-level)*sign<16.:
            continue
        pixels=np.rint((obj[:,:2]-origin)*2).astype(int)
        x,y=pixels.T
        within=(x>=0)&(y>=0)&(x<mask.shape[1])&(y<mask.shape[0])&(abs(obj[:,2]-level)<14.)
        ids=np.flatnonzero(within)
        surrounding=ids[(mask[y[ids],x[ids]]==0)&
                        (cv2.dilate(mask,np.ones((17,17),np.uint8))[y[ids],x[ids]]!=0)]
        ids=ids[mask[y[ids],x[ids]]!=0]
        if len(ids)<28:
            continue
        # Independent occupied CAD cells prevent oversampled narrow lips/jaws
        # from being interpreted as an observed end face.
        points=obj[ids]
        center=np.median(points[:,2])
        support=points[abs(points[:,2]-center)<2.5]
        if len(support)<28 or len(support)<.65*len(points):
            continue
        cells=np.unique(np.floor((support[:,:2]-origin)/3.),axis=0)
        if len(cells)<18 or np.any(np.ptp(support[:,:2],axis=0)<[18.,12.]):
            continue
        outside=obj[surrounding]
        same_plane=outside[abs(outside[:,2]-center)<2.5]
        if len(np.unique(np.floor((same_plane[:,:2]-origin)/3.),axis=0))>.35*len(cells):
            continue
        _,sv,axes=np.linalg.svd(support-support.mean(0),full_matrices=False)
        if sv[1]/np.sqrt(len(support))<3. or abs(axes[-1,2])<.97:
            continue
        # The rear and front are separated by the CAD length. Use the
        # measured centre of this terminal face for its axial position.
        surface=np.median(support,axis=0)
        surface[:2]=(bounds[0,:2]+bounds[1,:2])/2
        surface[2]+=bounds[0,2]-level
        return dict(point_base_m=(tool[:3,3]+rotation@(surface-contact)/1000).tolist(),
                    method='measured_rear_face_and_CAD_length' if sign==1 else 'measured_front_face',
                    support_pixels=len(support))
    raise ValueError('电池端面有效深度不足')


DIRECT_MM = 5.


def record_cross_track(state,delta_mm,axis):
    """Keep the live offset and its peak relative to the original stroke line."""
    cross=float(np.linalg.norm(delta_mm-(delta_mm@axis)*axis))
    state['cross_track_mm']=cross
    state['peak_cross_track_mm']=max(cross,state.get('peak_cross_track_mm',0.))


def record_rotation(state,initial,actual):
    cosine=(np.trace(initial.T@actual)-1.)/2.
    angle=float(np.degrees(np.arccos(np.clip(cosine,-1.,1.))))
    state['rotation_drift_deg']=angle
    state['peak_rotation_drift_deg']=max(angle,state.get('peak_rotation_drift_deg',0.))


def axis_basis(direction):
    axis=np.asarray(direction,float)
    if axis.shape!=(3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1.)>1e-5:
        raise ValueError('入仓轴向无效')
    axis=axis/np.linalg.norm(axis)
    up=np.array([0.,0.,1.])-axis*axis[2]
    if np.linalg.norm(up)<1e-6:
        up=np.array([0.,1.,0.])-axis*axis[1]
    return axis,up/np.linalg.norm(up)


def lock_travel_axis(observation,axis):
    axis,_=axis_basis(axis)
    normal=np.asarray(observation['mouth']['inside_axis_base'],float)
    cosine=float(normal@axis)
    if not np.cos(np.radians(ENTRY_ANGLE_DEG))-1e-12<=cosine<=1.+1e-8:
        raise ValueError('选定推进轴与实测仓口夹角超过 10°，保持夹持')
    return dict(observation,axis_base=axis.tolist(),travel_cosine=cosine,
                gap_mm=-observation['body_depth_mm']/cosine)


def axial_gap(cabinet, point):
    n=np.asarray(cabinet['inside_axis_base'],float)
    p=np.asarray(cabinet['mouth_anchor_base_m'],float)
    front=np.asarray(point,float)
    if not np.isfinite(np.r_[n,p,front]).all() or n[0]<.85:
        raise ValueError('仓口深度平面无效')
    return float((p-front)@n/n[0]*1000)


def seating_geometry(mouth, nose, tool, grasp_span_mm, *, seat_only=False):
    normal=np.asarray(mouth['inside_axis_base'],float)
    forward=tool[:3,0]
    cosine=float(normal@forward)
    if not seat_only and cosine<np.cos(np.radians(ENTRY_ANGLE_DEG))-1e-12:
        angle=np.degrees(np.arccos(np.clip(cosine,-1.,1.)))
        raise ValueError(f'夹爪纵轴与仓口夹角 {angle:.2f}°，超过 10°')
    depth=float((np.asarray(nose)-mouth['mouth_anchor_base_m'])@normal)*1000.
    gap=None if seat_only else -depth/cosine
    # The CAD tip landmark is the innermost point of the taught handle
    # grasp. Anchor its predicted position to the depth-observed battery.
    offset=(np.asarray(nose)-tool[:3,3])*1000.
    transverse=offset-forward*(offset@forward)
    outside=grasp_span_mm*cosine+float(transverse@normal)-depth
    return dict(gap_mm=gap,body_depth_mm=float(depth),travel_cosine=cosine,axis_base=forward.tolist(),
                handle_outside_mm=float(outside),
                entry_angle_deg=float(np.degrees(np.arccos(np.clip(cosine,-1.,1.)))),mouth=mouth)


def placement_timing(limits,base_speed,terminal):
    cruise_speed = min(limits['tcp_speed_mm_s'], 5. * base_speed)
    cruise_accel = min(limits['tcp_acceleration_mm_s2'], 100.)
    # Contact approach uses a slower time scale, including if the global limits are reduced.
    speed = min(24., .6 * cruise_speed) if terminal else cruise_speed
    accel = .36 * cruise_accel if terminal else cruise_accel
    return dict(limits, tcp_speed_mm_s=speed, tcp_acceleration_mm_s2=accel)


def reobserve_seat(read,guard,report,wait):
    from agent.recovery_watch import RetryWatch
    watch=RetryWatch(None,'D405 坐底确认')
    attempt=0
    while True:
        guard()
        attempt+=1
        evidence=read()
        guard()
        if evidence.get('insertion_depth_verified') or evidence.get('allow_compensation'):
            return evidence
        if evidence.get('allow_compensation') is False and evidence.get('remaining_mm',0.) < -2.:
            return evidence  # Already past the taught depth. Never command a further push.
        reason=evidence.get('verification_reason','D405 等待识别')
        if '未找到当前完全入仓标准' in reason or '坐底标准使用的夹爪或电池标定已变化' in reason:
            raise ValueError(reason)
        report(attempt,reason)
        watch.fail(reason,evidence=evidence)
        wait(.1)


def capture_frame(capture,guard=None):
    if callable(getattr(capture,'live_frame',None)):
        return capture.live_frame(guard=guard)
    view=capture.action({'op':'capture'})
    with np.load(capture.cache/'frame.npz',allow_pickle=False) as stored:
        frame={k:stored[k] for k in stored.files}
    if str(frame['id'])!=view['id']:
        raise ValueError('深度曝光已被替换')
    return frame


def measure_approach(captures, controller, tcp, guard, *,root=ROOT,seat_only=False):
    if seat_only:
        from seated_reference import measure_seated_reference
        return measure_seated_reference(captures['d405'],controller,tcp,guard,root,lambda c:capture_frame(c,guard))
    rows={};samples=[];poses=[];stamps=[];partial_planes={}
    for name in ('d405','d435'):
        guard()
        try:
            f=capture_frame(captures[name])
            if str(f['camera_id'])!=name:
                raise ValueError('相机身份不符')
            stamp=float(f['image_time'])
            if not -.05<=time.time()-stamp<=2. or max(abs(stamp-float(f[k])) for k in ('arm_time','body_time'))>.2:
                raise ValueError('曝光与关节反馈时间不匹配')
            tool=controller.models['right'].wrist_transform(f['joints'],float(f['lift_rad']))@tcp
            if name=='d405':
                rgb,xyz,T,rs,intr=paired_geometry(f)
            else:
                cal=HEAD_CALIBRATION.read_bytes()
                if (str(f['calibration_sha256'])!=hashlib.sha256(cal).hexdigest() or
                    not np.allclose(f['base_from_color'],json.loads(cal)['T_base_from_color_optical'])):
                    raise ValueError('D435 外参不匹配')
                rgb,xyz,T=f['rgb'],f['xyz'],f['base_from_color']
                rs,intr=camera_intrinsics(json.loads(str(f['intrinsics'])))
            poses.append(tool);stamps.append(stamp)
            row=rows[name]=dict(frame_id=str(f['id']),image_time=stamp)
            try:row['mouth']=insertion_mouth(rgb,xyz,T,rs,intr)
            except ValueError as e:
                row['mouth_reason']=str(e)
                try:
                    mask,_,valid,n,d,nb,_=_rim_plane(rgb,xyz,T)
                    cloud=xyz[mask & valid]
                    near=cloud[np.abs(cloud@n-d)<.003]@T[:3,:3].T+T[:3,3]
                    axis=nb*np.sign(nb[0])
                    anchor=T[:3,:3]@(n*d)+T[:3,3]
                    partial_planes[name]=(dict(inside_axis_base=axis,mouth_anchor_base_m=anchor),near)
                except ValueError:pass
            try:row['front']=measured_end(f,tool,root)
            except ValueError as e:
                row['front_reason']=str(e)
                if name=='d405' and 'mouth' in row:
                    try: row['front']=measured_wrist_front(f,tool,row['mouth'],root)
                    except ValueError as err:row['front_reason']=str(err)
            if 'front' in row and 'mouth' in row:
                samples.append((name,row['mouth'],row['front']['point_base_m']))
        except (ValueError,OSError,RuntimeError) as e:
            rows[name]=dict(reason=str(e))
    guard()
    if not poses:
        raise ValueError('两台深度相机均未取得配对曝光')
    current=controller.snapshot()
    now_tool=controller.models['right'].wrist_transform(current['arms']['right']['joints'],current['body']['height'])@tcp
    if (time.time()-min(stamps)>4. or not current['arms']['right']['fresh'] or not current['body']['fresh'] or
        any(np.linalg.norm(t[:3,3]-now_tool[:3,3])>.0015 or
            np.trace(t[:3,:3].T@now_tool[:3,:3])<1+2*np.cos(np.radians(.5)) for t in poses)):
        raise ValueError('采集期间末端位姿变化或深度帧已过期')
    # When a camera sees the battery and part of the red face, the other
    # camera can identify the mouth. Verify that both cameras see the same
    # physical plane before combining points across their extrinsics.
    for src,other in (('d435','d405'),('d405','d435')):
        body=rows[src].get('front');mouth=rows[other].get('mouth')
        if body is None or mouth is None or src not in partial_planes:continue
        plane,cloud=partial_planes[src]
        if (not len(cloud) or np.min(np.linalg.norm(cloud-mouth['mouth_anchor_base_m'],axis=1))>.030
                or np.dot(plane['inside_axis_base'],mouth['inside_axis_base'])<np.cos(np.radians(12.))):
            continue
        here=axial_gap(plane,body['point_base_m']);other_gap=axial_gap(mouth,body['point_base_m'])
        if abs(here-other_gap)<=5.:
            samples.append((src,mouth,body['point_base_m']))
            rows[src]['mouth_reference']=other
    if not samples:
        raise ValueError('无法计算仓外间距。'+'；'.join(k+' '+(v.get('reason') or '，'.join(v[n] for n in ('mouth_reason','front_reason') if n in v)) for k,v in rows.items()))
    bounds,contact=battery_geometry(root)
    length=contact[2]-bounds[0,2]
    measurements=[seating_geometry(mouth,nose,now_tool,length,seat_only=seat_only) for _,mouth,nose in samples]
    depths=[m['body_depth_mm'] for m in measurements]
    if np.ptp(depths)>5.:
        raise ValueError('两台相机的电池深度相差 %.1f mm'%np.ptp(depths))
    for m in measurements[1:]:
        other=measurements[0]['mouth']
        if (abs(axial_gap(other,m['mouth']['mouth_anchor_base_m']))>5. or
                np.dot(other['inside_axis_base'],m['mouth']['inside_axis_base'])<np.cos(np.radians(5.))):
            raise ValueError('两台相机的仓口平面不一致')
    chosen=int(np.argmin(abs(np.asarray(depths)-np.median(depths))))
    result=measurements[chosen]
    if (not seat_only and result['gap_mm']>150.) or result['body_depth_mm']>PUSH_MM+5.:
        raise ValueError('当前电池深度超出已标定的入仓范围')
    return dict(result,cameras=rows,source=samples[chosen][0],start_tool=now_tool.tolist())


def measured_wrist_front(frame,tool,cabinet,root):
    from d405_left_edges import _left_battery
    from insertion_observation import partial_front
    bounds,contact=battery_geometry(root)
    try:
        tracking=dict(axis_base=cabinet['inside_axis_base'],mouth_base_m=cabinet['mouth_anchor_base_m'],
                      quad_px=None,expected_front_base_m=None)
        front=battery_front(frame,cabinet,outside_range_mm=(-210.,300.),insertion_tracking=tracking)
        obj=(np.asarray(front['point_base_m'])-tool[:3,3])@(tool[:3,:3]@OBJECT_TO_TOOL)*1000+contact
        if not np.isfinite(obj).all() or abs(obj[2]-bounds[0,2])>10. or np.any(obj[:2]<bounds[0,:2]-10.) or np.any(obj[:2]>bounds[1,:2]+10.):
            raise ValueError('候选棱与电池前端不一致')
        return dict(front,method='observed_four_edges')
    except ValueError:
        rgb,xyz,T,_,intr=paired_geometry(frame)
        side=_left_battery(rgb,xyz,T,tool,bounds,contact,intr)
        return dict(partial_front(*side,tool,bounds,contact),method='observed_distal_edge')


def verify_fixed_depth(observation,capture,guard):
    """Require an independent exposure of the battery and the same mouth."""
    guard()
    began=time.time()
    try:
        post=capture()
    except (ValueError,OSError,RuntimeError,TimeoutError) as error:
        guard()
        return dict(insertion_depth_verified=False,verification_reason=str(error))
    guard()
    old=[r for r in observation.get('cameras',{}).values() if 'image_time' in r]
    frames=[r for r in post['cameras'].values() if 'image_time' in r]
    if (not frames or (old and min(r['image_time'] for r in frames)<=max(r['image_time'] for r in old))
            or any(r['image_time']<began or not -.05<=time.time()-r['image_time']<=4. or
                   r['frame_id'] in {v['frame_id'] for v in old} for r in frames)):
        return dict(insertion_depth_verified=False,verification_reason='未取得推入后的新曝光')
    prior=observation.get('mouth');plane=post['mouth']
    if prior is not None and (abs(axial_gap(prior,plane['mouth_anchor_base_m']))>5. or
            np.dot(prior['inside_axis_base'],plane['inside_axis_base'])<np.cos(np.radians(5.))):
        return dict(insertion_depth_verified=False,verification_reason='复查仓口平面发生变化')
    depth=post['body_depth_mm'];outside=post['handle_outside_mm']
    cosine=float(np.dot(observation['axis_base'],plane['inside_axis_base']))
    if not observation.get('rail_guided') and not np.cos(np.radians(ENTRY_ANGLE_DEG))-1e-12<=cosine<=1.+1e-8:
        return dict(insertion_depth_verified=False,verification_reason='末端复查的仓口方向不支持坐底确认')
    using_reference=post.get('source')=='d405_operator_seated_reference'
    partial=post.get('source')=='d405_partial_body'
    seated=bool(not partial and abs(depth-PUSH_MM)<=2. and outside>0. and (not using_reference or post['reference_match']))
    details={}
    if using_reference:
        details={k:post[k] for k in ('reference_id','reference_match','allow_compensation','remaining_mm',
                                     'shoulder_errors_mm','handle_error_mm','source')}
    reason=('本体两侧及把手符合完全入仓标准' if seated else '尚未匹配完全入仓标准') if using_reference else (
        '本体已到仓底，把手留在仓外' if seated else '视觉尚未确认本体坐底且把手在仓外')
    if partial:
        details={k:post[k] for k in ('source','allow_compensation','compensation_limit_mm')}
        reason='D405 已量到仓内电池本体，按剩余深度补推' if post['allow_compensation'] else '正在等待坐底画面'
    return dict(details,insertion_depth_verified=seated,verified_depth_mm=depth,handle_outside_mm=outside,
                travel_cosine=cosine,verification_mouth=plane,verification_image_time=min(r['image_time'] for r in frames),
                verification_frame_ids=[r['frame_id'] for r in frames],
                verification_reason=reason)


def run_mouth_push(observation,read_tool,move,guard,verify,*,travel_budget_mm=300.):
    """Cross the mouth without stopping, then correct a measured shortfall."""
    start=np.asarray(observation['start_tool'],float)
    depth=float(observation['body_depth_mm'])
    axis,_=axis_basis(observation['axis_base'])
    cosine=float(observation['travel_cosine'])
    if (not np.isfinite([depth,cosine]).all() or not np.cos(np.radians(ENTRY_ANGLE_DEG))-1e-12<=cosine<=1.000000001
            or -depth/cosine>150. or depth>PUSH_MM+2.):
        raise ValueError('实测间距或入仓朝向不在可执行范围')
    initial=0. if abs(depth-PUSH_MM)<=2. else (PUSH_MM-depth)/cosine
    return _push_and_verify(start,axis,initial,observation,read_tool,move,guard,verify,travel_budget_mm)


def axial_travel_mm(start,actual,axis):
    travel=float((actual[:3,3]-start[:3,3])@axis)*1000.
    if not np.isfinite(travel):
        raise ValueError('轴向行程反馈无效')
    return travel


def fresh_depth_evidence(result):
    timestamp=result.get('verification_image_time')
    return (bool(result.get('verification_frame_ids')) and isinstance(timestamp,(int,float))
            and -.05<=time.time()-timestamp<=4.)


def require_seated_release(start,actual,axis,result):
    measured=axial_travel_mm(start,actual,axis)
    contact=result.get('contact_stop')
    if contact is not None:
        stopped=contact.get('stopped_travel_mm')
        stamp=contact.get('stop_confirmed_at')
        image=result.get('verification_image_time')
        if (contact.get('stop_confirmed') is not True or contact.get('reference_reset') is not True
                or contact.get('kind') not in ('current_contact','progress_stall')
                or not isinstance(stopped,(int,float)) or not np.isfinite(stopped)
                or not isinstance(stamp,(int,float)) or not np.isfinite(stamp)
                or not isinstance(image,(int,float)) or not image>=stamp
                or not PUSH_MM-CONTACT_SEAT_ZONE_MM<=measured<=PUSH_MM+CONTACT_SEAT_ZONE_MM
                or abs(measured-stopped)>ARRIVAL_MM):
            raise ValueError('阻塞停止不在坐底区间或停稳/新视觉证据失效')
    elif measured+TRAVEL_EPS_MM<PUSH_MM:
        raise ValueError(f'轴向实测 {measured:.3f} mm，尚未达到 150 mm')
    depth=result.get('verified_depth_mm')
    outside=result.get('handle_outside_mm')
    if (not fresh_depth_evidence(result) or result.get('insertion_depth_verified') is not True
            or not isinstance(depth,(int,float)) or not abs(depth-PUSH_MM)<=2.
            or not isinstance(outside,(int,float)) or not np.isfinite(outside) or outside<=0.):
        raise ValueError('缺少本体坐底且把手在仓外的新深度观测')
    return measured


def run_fixed_stroke(start,read_tool,move,guard,verify,*,monitor=None):
    """Use one rail axis throughout. Only current, independent depth can justify extra travel."""
    axis,_=axis_basis(start[:3,0])
    observation=dict(rail_guided=True,axis_base=axis.tolist())
    phase='seat_push'
    length=PUSH_MM
    corrections=0
    extra=0.
    last_depth=None
    frames_seen=set()
    while True:
        guard()
        before=read_tool()
        before_mm=axial_travel_mm(start,before,axis)
        if before_mm+length>MAX_RAIL_TRAVEL_MM+TRAVEL_EPS_MM:
            raise ValueError('下一段超出 300 mm 总行程，保持夹持')
        contact=None
        try:
            with (monitor() if monitor is not None else nullcontext()):
                move(phase,length,before[:3,:3].copy())
        except InsertionContact as stopped:
            contact=stopped.evidence
            if contact.get('stop_confirmed') is not True or contact.get('reference_reset') is not True:
                raise ValueError('阻塞停止未确认，禁止松爪和继续推进') from stopped
        guard()
        reached=axial_travel_mm(start,read_tool(),axis)
        delta=reached-before_mm
        if (delta<=0. and contact is None) or delta < -ARRIVAL_MM or delta>length+ARRIVAL_MM:
            raise ValueError('实测推进方向或行程异常，保持夹持')
        # A short but acknowledged stroke is a reason to observe, never to replay its nominal distance.
        feedback=verify(observation,guard)
        guard()
        actual=read_tool()
        travel=axial_travel_mm(start,actual,axis)
        result=dict(feedback,release_ready=False,minimum_travel_mm=PUSH_MM,
                    minimum_travel_reached=travel+TRAVEL_EPS_MM>=PUSH_MM,measured_travel_mm=travel,
                    planned_initial_travel_mm=PUSH_MM,compensation_count=corrections,compensation_mm=extra)
        if contact is not None:
            result.update(contact_detected=True,contact_stop=contact,
                          next_phase='realign_required' if travel<PUSH_MM-CONTACT_SEAT_ZONE_MM else 'hold')
        ids=set(feedback.get('verification_frame_ids',[]))
        if not fresh_depth_evidence(feedback) or ids & frames_seen:
            result.update(insertion_depth_verified=False,verification_reason=(
                '复查深度画面已重复或过期' if ids else feedback.get('verification_reason','缺少新的深度画面')))
            return result
        frames_seen.update(ids)
        if feedback.get('insertion_depth_verified'):
            try:
                require_seated_release(start,actual,axis,result)
            except ValueError as error:
                result['verification_reason']=str(error)
            else:
                result['release_ready']=True
            return result
        if contact is not None:
            return dict(result,verification_reason='阻塞后未确认坐底，保持夹持并等待重新对齐')
        if feedback.get('allow_compensation') is False:
            return dict(result,verification_reason='本体两侧或把手证据不足以继续补推，保持夹持')
        depth=feedback.get('verified_depth_mm')
        outside=feedback.get('handle_outside_mm')
        partial=feedback.get('source')=='d405_partial_body' and feedback.get('allow_compensation') is True
        if (not isinstance(depth,(int,float)) or not np.isfinite(depth)
                or (not partial and (not isinstance(outside,(int,float)) or not np.isfinite(outside) or outside<=0.))
                or PUSH_MM-depth<=2.):
            return result
        if last_depth is not None and depth-last_depth<.5:
            return dict(result,verification_reason='补推后电池深度未增加，保持夹持')
        remaining=MAX_RAIL_TRAVEL_MM-travel
        if remaining<=ARRIVAL_MM:
            return dict(result,verification_reason='已到总行程上限，未确认坐底')
        if 'verification_mouth' in feedback:
            observation['mouth']=feedback['verification_mouth']
        observation['cameras']={str(i):dict(frame_id=frame_id,image_time=feedback['verification_image_time'])
                                for i,frame_id in enumerate(ids)}
        phase='compensate'
        length=min(10.,PUSH_MM-depth,remaining)
        if partial:
            cap=feedback.get('compensation_limit_mm')
            if not isinstance(cap,(int,float)) or not np.isfinite(cap) or cap<=0.:
                return dict(result,verification_reason='D405 尚未得到有效的剩余行程')
            length=min(length,cap)
        corrections+=1
        extra+=length
        last_depth=depth


def _push_and_verify(start,axis,initial,observation,read_tool,move,guard,verify,travel_budget_mm):
    rotation=start[:3,:3].copy()
    if not 0.<travel_budget_mm<=300. or initial>travel_budget_mm:
        raise ValueError('本次接近与推入行程超过 300 mm')
    travelled=0.;corrections=0;extra=0.;last_depth=None
    phase='seat_push';length=initial;previous_frames=set()
    while True:
        guard();before=read_tool()
        expected=start[:3,3]+axis*travelled/1000.
        delta_to_start=(before[:3,3]-expected)*1000.
        start_error=np.linalg.norm(delta_to_start)
        if (start_error>ARRIVAL_MM or
                np.trace(rotation.T@before[:3,:3])<1+2*np.cos(np.radians(1.))):
            raise ValueError('末端偏离测量位置或固定朝向，保持夹持')
        if length>0.:
            move(phase,length,rotation)
            guard();after=read_tool();delta=(after[:3,3]-before[:3,3])*1000.
            progress=float(delta@axis)
            if (abs(progress-length)>ARRIVAL_MM or np.linalg.norm(delta-progress*axis)>2.5 or
                    np.trace(rotation.T@after[:3,:3])<1+2*np.cos(np.radians(1.))):
                raise ValueError('末端反馈未到达本段终点，保持夹持')
            travelled+=progress
        feedback=verify(observation,guard)
        result=dict(feedback,planned_initial_travel_mm=initial,compensation_count=corrections,
                    compensation_mm=extra,measured_travel_mm=travelled)
        ids=set(feedback.get('verification_frame_ids',[]))
        if corrections and (not ids or ids & previous_frames):
            return dict(result,insertion_depth_verified=False,verification_reason='补推后未取得独立新曝光')
        previous_frames.update(ids)
        if 'verification_mouth' in feedback:
            observation=dict(observation,mouth=feedback['verification_mouth'])
        if result['insertion_depth_verified']:return result
        measured=feedback.get('verified_depth_mm')
        if measured is None or feedback.get('handle_outside_mm',0.)<=0.:return result
        shortfall=PUSH_MM-measured
        if not 2.<shortfall<=20. or corrections>=2:return result
        if last_depth is not None and measured-last_depth<.5:
            result['verification_reason']='补推后电池深度未增加，已停推并保持夹持'
            return result
        length=min(shortfall/feedback['travel_cosine'],10.,20.-extra,travel_budget_mm-travelled)
        if length<=0.:return result
        last_depth=measured;phase='compensate';corrections+=1;extra+=length
