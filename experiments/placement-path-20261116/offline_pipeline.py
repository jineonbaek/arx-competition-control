"""Offline-only perception and kinematic preview, never sends robot commands."""
import numpy as np
from agent.cabinet_preview_geometry import localize
from agent.visual_battery_level import d435_observation,visual_level_step
from agent.held_placement_geometry import effective_tcp
from agent.grasp_path import plan_correction
from agent.held_insertion_path import plan_insert
from agent.d405_aperture_alignment import estimate_vertical_alignment
from offline_replay import preview_shallow

def d405_paired(reference,frame):
    try:
        image=float(frame['image_time']);arm=float(frame['arm_time']);body=float(frame['body_time']);ref=float(reference['image_time'])
        return str(frame['camera_id'])=='d405' and np.isfinite([image,arm,body,ref]).all() and max(abs(image-ref),abs(image-arm),abs(image-body))<=.2
    except (KeyError,TypeError,ValueError):return False

def apply_d405_vertical(preview,window,slot,estimate):
    import copy,cv2
    if estimate.get('status')!='visual_guidance_only':raise ValueError('D405 alignment unavailable')
    step=float(estimate['step_mm'])
    if not np.isfinite(step) or abs(step)>3.:raise ValueError('D405 correction exceeds 3 mm')
    axis=np.asarray(preview['insertion_axis_base'],float);normal=np.asarray(window['outward_normal_base'],float)
    if axis.shape!=(3,) or normal.shape!=(3,) or abs(axis@normal)<.7:raise ValueError('Invalid entrance direction')
    shift=np.array([0.,0.,step]);shift=shift-axis*(shift@normal)/(axis@normal)
    out=copy.deepcopy(preview)
    for key in ('tcp_points_base_mm','tcp_pre_base_mm','tcp_end_base_mm','leading_points_base_mm'):
        out[key]=(np.asarray(out[key],float)+shift).tolist()
    samples=np.asarray(out['aperture_envelope_samples_mm'],float)
    samples+=np.array([shift@np.asarray(window['left_axis_base']),shift@np.asarray(window['up_axis_base'])])
    outline=np.asarray(slot['outline_mm'],np.float32)-np.asarray(slot['center_mm'],np.float32)
    outline[:,0]*=-1
    outline*=np.asarray(out['effective_opening_size_mm'],np.float32)/np.asarray(slot['size_mm'],np.float32)
    clearance=min(float(cv2.pointPolygonTest(outline,tuple(p),True)) for p in samples)
    if clearance<=0:raise ValueError('D405 offset would leave the nominal aperture')
    out['aperture_envelope_samples_mm']=samples.tolist();out['nominal_cad_clearance_mm']=clearance
    out['d405_vertical_step_mm']=step
    return out

def plan_offline(*,frame,pose_report,mesh_sha,model,tcp,slot,battery,contact,limits,timing_limits,d405_frame=None,right_offset_mm=0.):
    stamp=float(frame["image_time"]);q=np.asarray(frame["joints"],float);h=float(frame["lift_rad"]);tcp=np.asarray(tcp,float)
    result={"mode":"offline_replay","stage_order":[],"executable":False,"gripper_action":"retain","d405":None,"insert":None}
    window=localize(frame,profile=slot,now=stamp);result["stage_order"].append("cabinet");result["cabinet"]=window
    preview=preview_shallow(window,slot,battery,contact,right_offset_mm=right_offset_mm);result["geometric_preview"]=preview
    if d405_frame is not None and d405_paired(frame,d405_frame):
        try:result["d405"]=estimate_vertical_alignment(d405_frame)
        except ValueError as e:result["d405"]={"status":"unavailable","reason":str(e),"executable":False}
    else:result["d405"]={"status":"unavailable","reason":"recorded D405 RGB-D frame missing or not time-paired","executable":False}
    if result['d405'].get('status')=='visual_guidance_only':
        try:preview=apply_d405_vertical(preview,window,slot,result['d405']);result['geometric_preview']=preview
        except (ValueError,KeyError) as e:result['d405']={'status':'unavailable','reason':str(e),'executable':False}
    observation=d435_observation(frame,pose_report,mesh_sha)
    attachment=np.linalg.inv(model.wrist_transform(q,h)@tcp)@np.asarray(observation["T_base_from_object"],float)
    effective=effective_tcp(tcp,attachment,contact)
    level=visual_level_step(observation,model,q,h,tcp,limits,now=stamp,max_age_s=5.)
    result["stage_order"].append("level");result["level"]=level
    q_level=np.asarray(level.get("nominal_target_rad",q),float)
    if not model.check(q,q_level,h)["clear"]:
        result["align"]={"status":"unavailable","reason":"leveling arm/base geometry rejected"};return result
    target=np.eye(4);target[:3,:3]=np.asarray(preview["R_base_from_tool"],float);target[:3,3]=np.asarray(preview["tcp_pre_base_mm"],float)/1000.
    result["stage_order"].append("align")
    try:align=plan_correction(model,q_level,h,effective,target,limits,timing_limits=timing_limits,translation_step_mm=5.,rotation_step_deg=2.)
    except ValueError as e:
        result["align"]={"status":"unavailable","reason":str(e)};return result
    result["align"]=align;q_aligned=np.asarray(align["joint_waypoints_rad"][-1],float)
    result["stage_order"].append("insert")
    try:
        result["insert"]=plan_insert(model,q_aligned,h,preview["tcp_pre_base_mm"],preview["tcp_end_base_mm"],preview["R_base_from_tool"],effective,limits,timing_limits=timing_limits,fixed_z_axis=preview["insertion_axis_base"])
    except ValueError as e:result["insert"]={"status":"unavailable","reason":str(e),"executable":False}
    return result
