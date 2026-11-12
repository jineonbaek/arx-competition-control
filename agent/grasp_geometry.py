"""Frozen metric evidence and a tool-contact preview; never a wrist command."""
import hashlib
import json
from pathlib import Path
import numpy as np
from foundationpose.battery_grip import transform_reference

ROOT=Path(__file__).resolve().parents[1]
PROFILE=ROOT/"web/public/cad/battery-grip.json"

def rigid(value):
    T=np.asarray(value,float)
    if (T.shape!=(4,4) or not np.isfinite(T).all() or
        not np.allclose(T[3],[0,0,0,1]) or
        not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-3) or np.linalg.det(T[:3,:3])<.99):
        raise ValueError("Invalid rigid transform")
    return T

def validate_frame(f):
    xyz,rgb,raw=np.asarray(f["xyz"]),np.asarray(f["rgb"]),np.asarray(f["raw_depth"])
    k=json.loads(str(f["intrinsics"]))
    shape=(k["height"],k["width"])
    if xyz.shape!=(*shape,3) or rgb.shape!=(*shape,3) or raw.shape!=shape or rgb.dtype!=np.uint8 or raw.dtype!=np.uint16:
        raise ValueError("RGB-D dimensions or format mismatch")
    if not np.isfinite(xyz).all() or not np.isfinite(float(f["scale"])) or float(f["scale"])<=0:
        raise ValueError("Invalid depth units or points")
    if any(not np.isfinite(k[key]) for key in ("fx","fy","cx","cy")) or min(k["fx"],k["fy"])<=0:
        raise ValueError("Invalid camera intrinsics")
    for key in ("image_time","arm_time","body_time","lift_rad"):
        if not np.isfinite(float(f[key])):raise ValueError("Invalid paired feedback")
    if max(abs(float(f["image_time"])-float(f[key])) for key in ("arm_time","body_time"))>.20:
        raise ValueError("Unpaired image and robot feedback")
    camera=str(f["camera_id"]) if "camera_id" in f else "d405"
    if camera not in ("d435","d405"):raise ValueError("Unknown geometry camera")
    valid=(xyz[...,2]>.07)&(xyz[...,2]<(3. if camera=="d435" else 1.5))
    if np.count_nonzero(valid)<100 or not np.allclose(xyz[...,2][valid],raw[valid]*float(f["scale"]),atol=1e-5):
        raise ValueError("Point cloud and raw depth disagree")
    return rigid(f["base_from_color"])

def preview_geometry(f,mask,pose,clearance_mm):
    base=validate_frame(f)
    if not np.isfinite(clearance_mm) or not 0<clearance_mm<=200:
        raise ValueError("Pregrasp clearance must be in (0,200] mm")
    if pose.get("id")!=str(f["id"]) or pose.get("frame")!="base_link":
        raise ValueError("Pose does not belong to this frozen frame")
    object_camera=rigid(pose["T_camera_from_object"])
    object_base=rigid(pose["T_base_from_object"])
    if not np.allclose(base@object_camera,object_base,atol=1e-6):
        raise ValueError("Pose and snapshot coordinate transforms disagree")
    xyz,rgb=f["xyz"],f["rgb"]
    mask=np.asarray(mask)
    if mask.shape!=xyz.shape[:2] or not np.isfinite(mask).all():raise ValueError("Mask dimensions invalid")
    mask=mask>0
    valid=(xyz[...,2]>.07)&(xyz[...,2]<1.5)
    selected=mask&valid
    if selected.sum()<100:raise ValueError("Insufficient battery surface depth")
    profile=json.loads(PROFILE.read_text())
    grip=transform_reference(profile,object_base,object_camera)
    target=np.asarray(grip["points_base_mm"]["grip"])
    direction=np.asarray(grip["approach_base"],float)
    pre=target-clearance_mm*direction
    # shortcut: task-space line only; wrist IK and swept gripper collisions wait for calibrated TCP.
    points=np.linspace(pre,target,max(2,int(np.ceil(clearance_mm/5))+1))
    yy,xx=np.nonzero(valid[::4,::4]);yy*=4;xx*=4
    cloud=xyz[yy,xx]@base[:3,:3].T+base[:3,3]
    colors=rgb[yy,xx].copy();colors[mask[yy,xx]]=[0,210,230]
    surface=xyz[selected]@base[:3,:3].T+base[:3,3]
    return dict(frame="base_link",unit="mm",snapshot_id=str(f["id"]),
        image_time=float(f["image_time"]),base_from_color=base.tolist(),
        points=np.column_stack([cloud,colors]).tolist(),grip=grip,
        path=dict(control_point="CAD grip reference G, not wrist",points_mm=points.tolist(),
                  pre_mm=pre.tolist(),end_mm=target.tolist(),kind="battery_approach_axis",
                  direction_base=direction.tolist(),alignment="follows FoundationPose battery axis",
                  required_j6_deg=-90),
        depth=dict(mask_pixels=int(mask.sum()),valid_pixels=int(selected.sum()),
            valid_ratio=float(selected.sum()/max(1,mask.sum())),
            nearest_surface_to_grip_mm=float(np.linalg.norm(surface*1000-target,axis=1).min())),
        profile_sha256=hashlib.sha256(PROFILE.read_bytes()).hexdigest(),
        tcp_status="pending",wrist_pregrasp_mm=None,wrist_grip_mm=None,
        executable=False,ik_checked=False,whole_path_collision_checked=False,
        blockers=["TCP 未接入","尚未验证目标朝向与水平夹姿匹配","尚未进行 IK 与夹爪全路径碰撞检查"],
        note="点云用于实测表面与 CAD 对照；孔中心可能是背景。当前线段为接触点 G 的几何模板，不是腕部轨迹。")
