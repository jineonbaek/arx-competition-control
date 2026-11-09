"""Grade stationary right-hand D405 observations from the existing capture owner."""
import json
import re
import sys
import time
from pathlib import Path
import cv2
import numpy as np

PUBLIC = Path(__file__).resolve().parents[1]/"public"
DICT = {s:getattr(cv2.aruco,"DICT_"+s) for s in ("4X4_50","4X4_100","4X4_250","5X5_50","5X5_100","5X5_250","6X6_250")}

LARGE_ROOT = PUBLIC.parents[1] / "vision/handeye/d405-aprilgrid-6x6"
LARGE_BOARD = json.loads((LARGE_ROOT / "board.json").read_text())

def board_spec(data):
    name=data.get("dictionary")
    marker=data.get("marker")
    mm=data.get("mm")
    if data.get("layout") == LARGE_BOARD["layout"]:
        if name != LARGE_BOARD["dictionary"] or mm != LARGE_BOARD["mm"]:
            raise ValueError("大板需要 tag36h11、55 mm 黑框")
        return dict(LARGE_BOARD)
    if name not in DICT or type(marker)!=int or not 0<=marker<int(name.split("_")[1]) or type(mm) not in (float,int) or not 1 <= mm <= 300:
        raise ValueError("请填写板上同一枚 ArUco 的实际字典、编号和黑色外框边长（毫米）")
    spec={"dictionary":name,"marker":marker,"mm":mm}
    layout=data.get("layout")
    if layout:
        if layout!="CC200-15-11.25" or (name,marker,mm)!=("5X5_100",26,11.25):
            raise ValueError("整板模式需要 CC200-15-11.25、5X5_100、参考码 26")
        spec["layout"]=layout
    return spec

class WaitForFrame(ValueError):
    """No time-matched post-settle frame yet."""


def estimate_target(image,k,spec):
    large = spec.get("layout") == LARGE_BOARD["layout"]
    if large:
        if not k or k.get("serial") != spec["serial"]:
            raise ValueError("D405 彩色内参缺失或序列号不匹配")
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters_create()
        params.markerBorderBits = spec["border_bits"]
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=params)
    else:
        corners,ids,_=cv2.aruco.detectMarkers(image,cv2.aruco.getPredefinedDictionary(DICT[spec["dictionary"]]))
    found=[] if ids is None else ids.ravel().tolist()
    whole_board=large or spec.get("layout")=="CC200-15-11.25"
    half=spec["mm"]/2000
    square=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],float)
    if whole_board:
        objects=[];pixels=[];markers=[]
        for marker,corner in zip(found,corners):
            if found.count(marker)!=1 or not 0 <= marker < (36 if large else 54):
                continue
            row,col=divmod(marker,6)
            if large:
                center=np.array([col-2.5,row-2.5,0])*spec["pitch_mm"]/1000
                # On this printed board OpenCV corner 0 is the lower right.
                local_square = -square
            else:
                center=np.array([2*col+(row+1)%2-5,4-row,0])*.015
                local_square = square
            objects.append(local_square+center)
            pixels.append(corner.reshape(4,2))
            markers.append(marker)
        if len(markers)<3:raise ValueError("整板至少需要三个已识别标记")
        obj=np.concatenate(objects);uv=np.concatenate(pixels)
    else:
        if found.count(spec["marker"])!=1:
            raise ValueError(f"未唯一识别指定标记 {spec['marker']}；本帧识别到 {found}")
        uv=corners[found.index(spec["marker"])].reshape(4,2)
        if np.linalg.norm(uv-np.roll(uv,1,axis=0),axis=1).min()<20:
            raise ValueError("标记在画面中过小（最短边低于 20 px）")
        obj=square;markers=[spec["marker"]]
    if not k or image.shape!=(k["height"],k["width"]):
        raise ValueError("缺少本帧 D405 彩色内参")
    dist=np.array(k["coeffs"],float)
    K=np.array([[k["fx"],0,k["cx"]],[0,k["fy"],k["cy"]],[0,0,1]],dtype=float)
    if k["model"]=="distortion.inverse_brown_conrady" and np.any(dist):
        # Use the factory model directly to obtain undistorted image coordinates.
        sdk=PUBLIC.parents[1]/"vision/vendor-python"
        if str(sdk) not in sys.path:sys.path.insert(0,str(sdk))
        import pyrealsense2 as rs
        intr=rs.intrinsics()
        intr.width,intr.height=k["width"],k["height"]
        intr.fx,intr.fy=k["fx"],k["fy"]
        intr.ppx,intr.ppy=k["cx"],k["cy"]
        intr.coeffs=k["coeffs"];intr.model=rs.distortion.inverse_brown_conrady
        rays=np.array([rs.rs2_deproject_pixel_to_point(intr,p.tolist(),1.) for p in uv])
        uv=rays[:,:2]*[intr.fx,intr.fy]+[intr.ppx,intr.ppy]
        dist=np.zeros(5)
    elif k["model"]!="distortion.brown_conrady" and np.any(dist):
        raise ValueError("当前 D405 畸变模型需先适配")
    if whole_board:
        flat=cv2.undistortPoints(uv.reshape(-1,1,2),K,dist,P=K)
        H,inliers=cv2.findHomography(obj[:,:2],flat,cv2.RANSAC,3.)
        if H is None or inliers is None:raise ValueError("整板标记几何不一致")
        mask=inliers.ravel().astype(bool)
        keep=mask.reshape(-1,4).sum(axis=1)>=3
        centers=np.asarray(objects)[keep].mean(axis=1)
        if keep.sum()<max(3,len(markers)*0.5) or np.linalg.matrix_rank(centers-centers.mean(0))<2:
            raise ValueError("整板有效标记过少或集中在一条线上")
        mask &= keep.repeat(4)
        obj=obj[mask];uv=uv[mask]
        markers=np.asarray(markers)[keep].tolist()
        ok,rv,tv=cv2.solvePnP(obj,uv,K,dist,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:raise ValueError("整板位姿求解失败")
        if np.min((obj@cv2.Rodrigues(rv)[0].T)[:,2]+tv[2,0])<=0:
            raise ValueError("整板位姿深度无效")
    else:
        ok,rv,tv=cv2.solvePnP(obj,uv.astype(float),K,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not np.isfinite(tv).all() or tv[2,0]<=0:raise ValueError("标记位姿求解失败")
    projected=cv2.projectPoints(obj,rv,tv,K,dist)[0].reshape(-1,2)
    rms=float(np.sqrt(np.mean(np.sum((uv-projected)**2,axis=1))))
    if not np.isfinite(rms) or rms>2:raise ValueError("重投影误差超过 2 px")
    T=np.eye(4);T[:3,:3]=cv2.Rodrigues(rv)[0];T[:3,3]=tv[:,0]
    return T,rms,markers


def inspect_large_board():
    """Inspect distinct D405 color frames without touching depth, robot state or sample files."""
    folder = PUBLIC / "camera-live"
    seen, transforms, counts, errors = set(), [], [], []
    session = None
    reason = "等待 D405 新鲜彩色帧"
    deadline = time.monotonic() + 6
    while len(seen) < 8 and time.monotonic() < deadline:
        time.sleep(.08)
        try:
            meta = json.loads((folder / "d405.json").read_text())
            name = meta.get("raw_image_file", "")
            if (meta.get("state") != "online" or not -.05 <= time.time()-meta.get("captured_at",0) < .8
                    or not re.fullmatch(r"d405-v-[a-f0-9]+-\d+\.jpg", name) or name in seen):
                continue
            if session is not None and meta["capture_session"] != session:
                raise ValueError("D405 采集会话已变化，请重新检查")
            session = meta["capture_session"]
            raw = (folder / name).read_bytes()
        except FileNotFoundError:
            continue
        seen.add(name)
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
        try:
            if image is None:
                raise ValueError("彩色快照无法解码")
            transform, rms, ids = estimate_target(image, meta.get("color_intrinsics"), LARGE_BOARD)
        except ValueError as error:
            reason = str(error)
            continue
        transforms.append(transform)
        counts.append(len(ids))
        errors.append(rms)
    spread = rotation_spread = None
    if transforms:
        xyz = np.array(transforms)[:,:3,3] * 1000
        spread = float(np.linalg.norm(xyz[:,None] - xyz, axis=2).max())
        rotation_spread = max(float(np.max(np.abs(a[:3,:3]-b[:3,:3])))
                              for a in transforms for b in transforms)
    stable = len(transforms) == 8 and spread <= 5 and rotation_spread <= .05
    return dict(frames=len(seen), usable=len(transforms), stable=stable, marker_counts=counts,
                rms_px=errors, spread_mm=spread, reason=None if stable else
                ("位姿波动超过静止阈值" if len(transforms)==8 else reason))


def observe(spec, after, arm, model, history=None, *, color_only=True):
    if not spec:raise ValueError("尚未设定标记字典、编号和实际边长")
    folder=PUBLIC/"camera-live"
    meta=json.loads((folder/"d405.json").read_text())
    stamp=meta.get("captured_at",0)
    name=meta.get("raw_image_file","")
    if history:
        ts,q=min(history,key=lambda item:abs(item[0]-stamp))
        arm={"timestamp":ts,"joints":q.tolist()}
    if meta.get("state")!="online" or not after<=stamp<=time.time()+.05 or time.time()-stamp>.8 or abs(stamp-arm["timestamp"])>.20 or not re.fullmatch(r"d405-v-[a-f0-9]+-\d+\.jpg",name):
        raise WaitForFrame("等待停稳后的新 D405 帧与同一时刻右臂反馈")
    if color_only and meta.get("depth",{}).get("state")!="disabled":
        raise ValueError("等待 D405 采集程序确认已切至彩色模式")
    data=(folder/name).read_bytes()
    image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_GRAYSCALE)
    k=meta.get("color_intrinsics")
    T,rms,markers=estimate_target(image,k,spec)
    wrist=np.linalg.inv(model.lift) @ model.pose(arm["joints"],0)[1]
    return {"frame":name,"camera_session":meta["capture_session"],"image_time":stamp,"arm_time":arm["timestamp"],
            "joints_rad":arm["joints"],"color_intrinsics":k,"board":spec,"inlier_marker_ids":markers,"rms_px":rms,
            "T_camera_from_marker":T.tolist(),"T_body4_from_right_link26":wrist.tolist()},data
