"""Immutable scene evidence from the existing console and its D405 capture owner."""
import base64
import io
import json
from pathlib import Path
import sys
from urllib.request import Request, urlopen
import uuid

import numpy as np

from .contracts import AgentError
from .storage import atomic_json

MAIN = Path.home() / "arx-competition-control"
VISION = Path.home() / "arx-vision-demo/vision/demo"
GRASP = Path.home() / "arx-graspnet-demo/grasp"


def capture(registry, camera, rgbd):
    if not rgbd:
        return registry.observations.capture([camera])
    if camera != "d405":
        raise AgentError("geometry_unavailable", "已配对的 RGB-D 和手眼后端当前支持右腕 D405")
    url = registry.config.console_url
    client = str(uuid.uuid4())
    request = Request(url + "/api/grasp-preview", data=b'{"op":"capture"}',
                      headers={"Origin": url, "Cookie": "arx_web_session=" + client,
                               "Content-Type": "application/json"})
    with urlopen(request, timeout=25) as response:
        view = json.load(response)
    blob = (MAIN / "web/hardware/runtime/grasp-preview/frame.npz").read_bytes()
    with np.load(io.BytesIO(blob), allow_pickle=False) as frame:
        if str(frame["id"]) != view["id"]:
            raise AgentError("stale_snapshot", "D405 采集结果已被更新，请重新采集")
        calibration = json.loads(str(frame["intrinsics"]))
        arm_time = float(frame["arm_time"])
    result = registry.store.save_observation({
        "source": "9983_d405_paired", "source_snapshot": view["id"],
        "arm_timestamp": arm_time, "target_frame": "body4",
        "cameras": {camera: {"image_size_px": [view["width"], view["height"]],
            "captured_at": view["time"], "intrinsics": calibration,
            "paired_ms": view["paired_ms"], "geometry_ready": True}}},
        {camera: base64.b64decode(view["image"].split(",", 1)[1], validate=True)})
    directory = registry.store.image_path(result["observation_id"], camera).parent
    (directory / "scene.npz").write_bytes(blob)
    return result


def scene(store, ident):
    store.observation(ident)
    directory = store.image_path(ident, "d405").parent
    if not (directory / "scene.npz").is_file():
        raise AgentError("geometry_unavailable", "请采集 D405 RGB-D 快照")
    return directory


def checked_pixel(rc, shape):
    row, col = rc
    if not (0 <= row < shape[0] and 0 <= col < shape[1]):
        raise AgentError("invalid_pixel", "像素超出快照，格式为 [row,col]")
    return row, col


def pixels(store, ident, points):
    with np.load(scene(store, ident) / "scene.npz", allow_pickle=False) as f:
        xyz, transform = f["xyz"], f["body4_from_color"]
        values = []
        for rc in points:
            point = xyz[checked_pixel(rc, xyz.shape)]
            valid = np.isfinite(point).all() and point[2] > 0
            values.append({"row_col": rc, "body4_m": (transform[:3,:3] @ point +
                           transform[:3,3]).tolist() if valid else None})
        if not any(v["body4_m"] is not None for v in values):
            raise AgentError("depth_unavailable", "所选像素均无有效表面深度")
        return {"observation_id": ident, "frame": "body4", "points": values}


def triangulate(store, ids, points):
    if len(ids) != len(points) or len(set(ids)) != len(ids) or len(ids) < 2:
        raise AgentError("invalid_correspondence", "需要两个以上不同快照中的同一静止物理点")
    vendor=str(MAIN / "vision/vendor-python")
    if vendor not in sys.path:
        sys.path.insert(0,vendor)
    import pyrealsense2 as rs
    views = []
    for ident, rc in zip(ids, points):
        with np.load(scene(store, ident) / "scene.npz", allow_pickle=False) as f:
            row,col = checked_pixel(rc, f["xyz"].shape)
            k = json.loads(str(f["intrinsics"]))
            intr = rs.intrinsics()
            for attr,key in (("width","width"),("height","height"),("fx","fx"),("fy","fy"),("ppx","cx"),("ppy","cy"),("coeffs","coeffs")):
                setattr(intr,attr,k[key])
            intr.model = getattr(rs.distortion,k["model"].split(".")[-1])
            T = f["body4_from_color"]
            ray = T[:3,:3] @ rs.rs2_deproject_pixel_to_point(intr,[col,row],1.)
            views.append((T,intr,np.array([col,row]),ray/np.linalg.norm(ray)))
    projectors = [np.eye(3)-np.outer(ray,ray) for _,_,_,ray in views]
    matrix = sum(projectors)
    if np.linalg.cond(matrix)>1e4:
        raise AgentError("low_parallax","视角夹角不足")
    point = np.linalg.solve(matrix,sum(P@v[0][:3,3] for P,v in zip(projectors,views)))
    errors = []
    for T,intr,uv,_ in views:
        p = T[:3,:3].T @ (point-T[:3,3])
        if p[2]<=0:
            raise AgentError("invalid_geometry","点在相机后方")
        errors.append(float(np.linalg.norm(np.array(rs.rs2_project_point_to_pixel(intr,p.tolist()))-uv)))
    if max(errors)>3:
        raise AgentError("reprojection_error","重投影偏差超过预览阈值 3 px")
    return {"body4_m":point.tolist(),"reprojection_px":errors,"observations":ids}
