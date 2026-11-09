"""Read-only AprilTag center preview using the nominal head-camera installation."""
import base64
import json
from pathlib import Path
import re
import time

import cv2
import numpy as np

from handeye_column import NOMINAL_RAD_PER_M

ROOT = Path(__file__).resolve().parents[2]
CAD = ROOT / "vision/handeye/results/d435-cad-349622074226.json"
LIVE = ROOT / "web/public/camera-live"
TAG_M = .055


def locate_tags(image, k, cad, body, stamp):
    if k.get("serial") != cad["serial"] or image.shape[:2] != (k["height"], k["width"]):
        raise ValueError("相机序列号或彩色内参与当前画面不匹配")
    coeffs = np.asarray(k["coeffs"],float)
    if np.any(coeffs) and k["model"] != "distortion.brown_conrady":
        raise ValueError("当前彩色畸变模型与预览不兼容")
    camera = np.array([[k["fx"],0,k["cx"]],[0,k["fy"],k["cy"]],[0,0,1.]],float)
    corners,ids,_ = cv2.aruco.detectMarkers(image,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    found = [] if ids is None else ids.ravel().tolist()
    square = TAG_M/2*np.array([[-1,1,0],[1,1,0],[1,-1,0],[-1,-1,0]],float)
    base_from_color = np.array(cad["T_base_from_color_optical"])
    fresh = body.get("fresh") and abs(stamp-body.get("timestamp",0))<=.2 and np.isfinite(body.get("height",float("nan")))
    lift = np.array(cad["T_base_from_body4_at_zero"])
    if fresh:lift[2,3] += body["height"]/NOMINAL_RAD_PER_M
    tags=[]
    for key,corner in zip(found,corners):
        uv=corner.reshape(4,2)
        if found.count(key)!=1 or np.linalg.norm(uv-np.roll(uv,1,axis=0),axis=1).min()<20:continue
        ok,rv,t=cv2.solvePnP(square,uv.astype(float),camera,coeffs,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or not np.isfinite(t).all() or t[2,0]<=0:continue
        pred=cv2.projectPoints(square,rv,t,camera,coeffs)[0].reshape(4,2)
        rms=float(np.sqrt(np.mean(np.sum((pred-uv)**2,axis=1))))
        if not np.isfinite(rms) or rms>2:continue
        center=base_from_color@np.r_[t.ravel(),1.]
        tags.append(dict(id=key,base_mm=(center[:3]*1000).round(1).tolist(),rms_px=round(rms,2),
                         body4_mm=(np.linalg.solve(lift,center)[:3]*1000).round(1).tolist() if fresh else None))
        cv2.aruco.drawDetectedMarkers(image,[corner],np.array([[key]],np.int32))
    return found,sorted(tags,key=lambda row:row["id"])


def preview(body):
    m=json.loads((LIVE/"d435.json").read_text())
    name=m.get("raw_image_file","")
    if m.get("state")!="online" or not -.05<=time.time()-m.get("captured_at",0)<1 or not re.fullmatch(r"d435-v-[a-f0-9]+-\d+\.jpg",name):
        raise ValueError("D435 彩色画面离线或过期")
    image=cv2.imread(str(LIVE/name))
    if image is None:raise ValueError("快照已更新，请重新读取")
    cad=json.loads(CAD.read_text())
    found,tags=locate_tags(image,m.get("color_intrinsics") or {},cad,body,m["captured_at"])
    ok,jpg=cv2.imencode(".jpg",image)
    if not ok:raise ValueError("快照编码失败")
    return dict(stamp=m["captured_at"],detected=len(found),tags=tags,
                image="data:image/jpeg;base64,"+base64.b64encode(jpg).decode())
