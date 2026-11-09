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
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    # The touching corners of the large AprilGrid defeat native-scale quad detection.
    enlarged = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
    large_corners, large_ids, _ = cv2.aruco.detectMarkers(enlarged, dictionary)
    native_corners, native_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    native = [] if native_ids is None else native_ids.ravel().tolist()
    large_keys = [] if large_ids is None else large_ids.ravel().tolist()
    ambiguous = {key for key in set(native + large_keys)
                 if native.count(key) > 1 or large_keys.count(key) > 1}
    found = list(native)
    corners = list(native_corners)
    for key, large in zip(large_keys, large_corners):
        if key in native:
            continue
        corner = (large + .5) / 2 - .5
        cv2.cornerSubPix(gray, corner, (3, 3), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
        found.append(key)
        corners.append(corner)
    square = TAG_M/2*np.array([[-1,1,0],[1,1,0],[1,-1,0],[-1,-1,0]],float)
    base_from_color = np.array(cad["T_base_from_color_optical"])
    fresh = body.get("fresh") and abs(stamp-body.get("timestamp",0))<=.2 and np.isfinite(body.get("height",float("nan")))
    lift = np.array(cad["T_base_from_body4_at_zero"])
    if fresh:lift[2,3] += body["height"]/NOMINAL_RAD_PER_M
    tags=[]
    board_pixels = {}
    for key,corner in zip(found,corners):
        uv=corner.reshape(4,2)
        if key in ambiguous or np.linalg.norm(uv-np.roll(uv,1,axis=0),axis=1).min()<20:continue
        ok,rv,t=cv2.solvePnP(square,uv.astype(float),camera,coeffs,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or not np.isfinite(t).all() or t[2,0]<=0:continue
        pred=cv2.projectPoints(square,rv,t,camera,coeffs)[0].reshape(4,2)
        rms=float(np.sqrt(np.mean(np.sum((pred-uv)**2,axis=1))))
        if not np.isfinite(rms) or rms>2:continue
        center=base_from_color@np.r_[t.ravel(),1.]
        tags.append(dict(id=key,base_mm=(center[:3]*1000).round(1).tolist(),rms_px=round(rms,2),
                         body4_mm=(np.linalg.solve(lift,center)[:3]*1000).round(1).tolist() if fresh else None))
        if 0 <= key < 36:
            board_pixels[key] = uv.mean(axis=0)
        cv2.aruco.drawDetectedMarkers(image,[corner],np.array([[key]],np.int32))
    board = board_center(tags, board_pixels)
    if board:
        center = np.r_[np.array(board["base_mm"]) / 1000, 1.]
        optical = np.linalg.solve(base_from_color, center)
        pixel = cv2.projectPoints(optical[:3].reshape(1, 3), np.zeros(3), np.zeros(3),
                                  camera, coeffs)[0].reshape(2)
        cv2.drawMarker(image, tuple(np.rint(pixel).astype(int)), (0, 220, 255),
                       cv2.MARKER_CROSS, 18, 2)
    return found, sorted(tags, key=lambda row:row["id"]), board


def board_center(tags, pixels):
    # This board numbers each row from left to right, starting in the bottom row.
    by_id = {tag["id"]: np.asarray(tag["base_mm"]) for tag in tags}
    pairs = [(by_id[key] + by_id[35 - key]) / 2 for key in pixels
             if key < 18 and 35 - key in pixels]
    if len(pairs) < 2 or len(pixels) < 5:
        return None
    grid = np.float32([[key % 6, key // 6] for key in pixels])
    observed = np.float32(list(pixels.values()))
    transform, _ = cv2.findHomography(grid, observed)
    if transform is None:
        return None
    projected = cv2.perspectiveTransform(grid[None], transform)[0]
    errors = np.linalg.norm(projected - observed, axis=1)
    if not np.isfinite(errors).all() or errors.max() > 3:
        return None
    return dict(base_mm=np.median(pairs, axis=0).round(1).tolist(), pairs=len(pairs))


def preview(body):
    m=json.loads((LIVE/"d435.json").read_text())
    name=m.get("raw_image_file","")
    if m.get("state")!="online" or not -.05<=time.time()-m.get("captured_at",0)<1 or not re.fullmatch(r"d435-v-[a-f0-9]+-\d+\.jpg",name):
        raise ValueError("D435 彩色画面离线或过期")
    image=cv2.imread(str(LIVE/name))
    if image is None:raise ValueError("快照已更新，请重新读取")
    cad=json.loads(CAD.read_text())
    found,tags,board=locate_tags(image,m.get("color_intrinsics") or {},cad,body,m["captured_at"])
    ok,jpg=cv2.imencode(".jpg",image)
    if not ok:raise ValueError("快照编码失败")
    return dict(stamp=m["captured_at"],detected=len(set(found)),tags=tags,board=board,
                image="data:image/jpeg;base64,"+base64.b64encode(jpg).decode())
