"""Shared RGB-D rectification and cross-camera CAD surface evidence, no motion."""
import json
import cv2
import numpy as np
from agent.grasp_geometry import validate_frame, rigid
from foundationpose.pose_quality import assess_depth_fit


def rectify(rgb, depth, mask, k, rs):
    if (rgb.shape[:2]!=depth.shape or mask.shape!=depth.shape
            or depth.shape!=(k['height'],k['width']) or not np.isfinite(mask).all()):
        raise ValueError('RGB-D and mask dimensions disagree')
    intr=rs.intrinsics()
    intr.width,intr.height=k['width'],k['height']
    intr.fx,intr.fy,intr.ppx,intr.ppy=k['fx'],k['fy'],k['cx'],k['cy']
    intr.model=getattr(rs.distortion,k['model'].split('.')[-1]);intr.coeffs=k['coeffs']
    h,w=depth.shape
    rays=[rs.rs2_project_point_to_pixel(intr,[(x-k['cx'])/k['fx'],(y-k['cy'])/k['fy'],1.])
          for y in range(h) for x in range(w)]
    maps=np.asarray(rays,np.float32).reshape(h,w,2)
    if not np.isfinite(maps).all():raise ValueError('Invalid rectification map')
    color=cv2.remap(rgb,maps[...,0],maps[...,1],cv2.INTER_LINEAR)
    z=cv2.remap(depth,maps[...,0],maps[...,1],cv2.INTER_NEAREST)
    fg=cv2.remap((mask>0).astype(np.uint8),maps[...,0],maps[...,1],cv2.INTER_NEAREST).astype(bool)
    K=np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1]],float)
    return color,z,fg,K,maps,intr


def cross_camera_fit(head, mask, base_from_object, rs, render):
    if str(head['camera_id'])!='d435':raise ValueError('Expected frozen D435 evidence')
    base_from_head=validate_frame(head)
    head_from_object=np.linalg.inv(base_from_head)@rigid(base_from_object)
    k=json.loads(str(head['intrinsics']))
    color,z,fg,K,_,_=rectify(head['rgb'],head['raw_depth'].astype(np.float32)*float(head['scale']),mask,k,rs)
    h,w=z.shape
    rendered=render(K,h,w,head_from_object)
    quality=assess_depth_fit(fg,z,rendered,max_depth_m=3.)
    return dict(head_snapshot_id=str(head['id']),head_image_time=float(head['image_time']),
        T_head_from_object=head_from_object.tolist(),quality=quality,
        identity_confirmed=False,executable=False,
        note='跨相机 CAD/深度一致性筛查；依赖两次观察间底盘和目标不动。D435 CAD 外参未实测，不能证明身份或抓取精度。')
