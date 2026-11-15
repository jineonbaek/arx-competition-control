"""RGB/CAD planar pose for preview. No depth, camera, ROS or motor access."""
import json
import re
import time
import cv2
import numpy as np
from .cabinet_preview_geometry import detect
from .placement_geometry import rigid


def camera_matrix(k, shape=None):
    try:
        values=np.array([k[n] for n in ('fx','fy','cx','cy','width','height')],float)
        coeffs=np.array(k['coeffs'],float)
        if (not np.isfinite(values).all() or min(values[:2])<=0 or min(values[4:])<=0
                or coeffs.shape!=(5,) or not np.isfinite(coeffs).all()
                or (shape is not None and tuple(shape[:2])!=(k['height'],k['width']))):
            raise ValueError('彩色内参与图像尺寸或数值不符')
        supported=('distortion.none','distortion.brown_conrady','distortion.inverse_brown_conrady')
        if k['model'] not in supported or (np.any(coeffs) and k['model']!='distortion.brown_conrady'):
            raise ValueError('当前彩色畸变模型不支持RGB定位')
        return np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1.]],float),coeffs
    except (KeyError,TypeError) as e:raise ValueError('缺少有效彩色内参') from e


def solve_quad_pose(quad, intrinsics, size_mm):
    """Return all plausible IPPE poses, best pixel fit first. Units: metres."""
    k,dist=camera_matrix(intrinsics);q=np.asarray(quad,float);size=np.asarray(size_mm,float)
    if (q.shape!=(4,2) or not np.isfinite(q).all() or size.shape!=(2,)
            or not np.isfinite(size).all() or np.any(size<=0) or np.any(size>2000)):
        raise ValueError('仓口角点或毫米制模型尺寸无效')
    w,h=size/2000
    obj=np.array([[-w,h,0],[w,h,0],[w,-h,0],[-w,-h,0]],float)
    try:ok,rotations,translations,*_=cv2.solvePnPGeneric(obj,q,k,dist,flags=cv2.SOLVEPNP_IPPE)
    except cv2.error as e:raise ValueError('仓口PnP求解失败') from e
    candidates=[]
    if ok:
        for rv,tv in zip(rotations,translations):
            R=cv2.Rodrigues(rv)[0];t=tv.ravel();points=obj@R.T+t
            if (not np.isfinite(points).all() or np.min(points[:,2])<.07 or np.max(points[:,2])>3.
                    or R[:,2]@t>=0):continue
            predicted=cv2.projectPoints(obj,rv,tv,k,dist)[0].reshape(4,2)
            rms=float(np.sqrt(np.mean(np.sum((predicted-q)**2,axis=1))))
            if not np.isfinite(rms) or rms>2.:continue
            T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
            candidates.append(dict(T_color_from_opening=T.tolist(),reprojection_rms_px=rms))
    if not candidates:raise ValueError('仓口RGB重投影或正深度检查未通过')
    return sorted(candidates,key=lambda c:c['reprojection_rms_px'])


def localize_rgb(frame, profile, now=None):
    now=time.time() if now is None else now
    try:
        stamp=float(frame['image_time']);arm=float(frame['arm_time']);body=float(frame['body_time'])
        if (str(frame['camera_id']) not in ('d435','d405') or not str(frame['id'])
                or not re.fullmatch('[a-f0-9]{64}',str(frame['calibration_sha256']))
                or not np.isfinite([stamp,arm,body,now]).all() or not -.1<=now-stamp<=2.
                or max(abs(stamp-arm),abs(stamp-body))>.2):
            raise ValueError('彩色帧过期、身份无效或与反馈不配对')
        rgb=np.asarray(frame['rgb']);k=json.loads(str(frame['intrinsics']))
        camera_matrix(k,rgb.shape)
        if not frame.get('camera_serial') or k.get('serial')!=str(frame['camera_serial']):
            raise ValueError('相机序列号与定位标定不一致')
        T=rigid(frame['base_from_color'])
        detected=detect(rgb)
        if detected['status']!='complete':raise ValueError(detected['reason'])
        detected.pop('_red',None)
        candidates=solve_quad_pose(detected['inner_quad_px'],k,profile['size_mm'])
        best=np.array(candidates[0]['T_color_from_opening']);base=T@best
        inside=-base[:3,2];left=-base[:3,0];up=base[:3,1]
        if up[2]<.7 or np.linalg.norm(inside[:2])<.7 or left@np.cross([0,0,1],inside)<=0:
            raise ValueError('仓口姿态不是适合水平放入的立面')
        w,h=np.array(profile['size_mm'])/2000
        obj=np.array([[-w,h,0],[w,h,0],[w,-h,0],[-w,-h,0]])
        corners=obj@base[:3,:3].T+base[:3,3]
        plausible=[c for c in candidates if c['reprojection_rms_px']<=candidates[0]['reprojection_rms_px']+.5]
        spread=max(np.linalg.norm(np.array(c['T_color_from_opening'])[:3,3]-best[:3,3])*1000 for c in plausible)
        angles=[np.degrees(np.arccos(np.clip((np.trace(best[:3,:3].T@np.array(c['T_color_from_opening'])[:3,:3])-1)/2,-1,1))) for c in plausible]
        return {**detected,'status':'localized','localization_source':'rgb_cad_pnp','snapshot_id':str(frame['id']),
                'camera':str(frame['camera_id']),'image_time':stamp,'calibration_sha256':str(frame['calibration_sha256']),
                'center_base_m':base[:3,3].tolist(),'corners_base_m':corners.tolist(),
                'outward_normal_base':base[:3,2].tolist(),'inside_axis_base':inside.tolist(),
                'left_axis_base':left.tolist(),'up_axis_base':up.tolist(),
                'model_aperture_size_mm':list(profile['size_mm']),
                'reprojection_rms_px':candidates[0]['reprojection_rms_px'],'pose_candidates':candidates,
                'pose_ambiguous':len(plausible)>1,'candidate_translation_spread_mm':float(spread),
                'candidate_rotation_spread_deg':float(max(angles)),
                'scale_assumption':'红框内缘对应CAD净口；尚未实物核验尺寸与贴标偏移',
                'executable':False}
    except (KeyError,TypeError,json.JSONDecodeError) as e:raise ValueError('彩色定位帧元数据不完整') from e
