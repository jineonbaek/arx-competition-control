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

def board_spec(data):
    name=data.get("dictionary")
    marker=data.get("marker")
    mm=data.get("mm")
    if name not in DICT or type(marker)!=int or not 0<=marker<int(name.split("_")[1]) or type(mm) not in (float,int) or not 1 <= mm <= 300:
        raise ValueError("请填写板上同一枚 ArUco 的实际字典、编号和黑色外框边长（毫米）")
    return {"dictionary":name,"marker":marker,"mm":mm}

def observe(spec, after, arm, model):
    if not spec:raise ValueError("尚未设定标记字典、编号和实际边长")
    folder=PUBLIC/"camera-live"
    meta=json.loads((folder/"d405.json").read_text())
    stamp=meta.get("captured_at",0)
    name=meta.get("raw_image_file","")
    if meta.get("state")!="online" or not after<=stamp<=time.time()+.05 or time.time()-stamp>.8 or abs(stamp-arm["timestamp"])>.20 or not re.fullmatch(r"d405-v-[a-f0-9]+-\d+\.jpg",name):
        raise ValueError("等待停稳后的新 D405 帧与同一时刻右臂反馈")
    data=(folder/name).read_bytes()
    image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_GRAYSCALE)
    corners,ids,_=cv2.aruco.detectMarkers(image,cv2.aruco.getPredefinedDictionary(DICT[spec["dictionary"]]))
    found=[] if ids is None else ids.ravel().tolist()
    if found.count(spec["marker"])!=1:
        raise ValueError(f"未唯一识别指定标记 {spec['marker']}；本帧识别到 {found}")
    uv=corners[found.index(spec["marker"])].reshape(4,2)
    if np.linalg.norm(uv-np.roll(uv,1,axis=0),axis=1).min()<20:
        raise ValueError("标记在画面中过小（最短边低于 20 px）")
    k=meta.get("color_intrinsics")
    if not k or image.shape!=(k["height"],k["width"]):
        raise ValueError("已识别标记，缺少本帧 D405 彩色内参；不能作为标定样本")
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
    half=spec["mm"]/2000
    obj=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],float)
    ok,rv,tv=cv2.solvePnP(obj,uv.astype(float),K,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not np.isfinite(tv).all() or tv[2,0]<=0:raise ValueError("标记位姿求解失败")
    projected=cv2.projectPoints(obj,rv,tv,K,dist)[0].reshape(4,2)
    rms=float(np.sqrt(np.mean(np.sum((uv-projected)**2,axis=1))))
    if not np.isfinite(rms) or rms>2:raise ValueError("重投影误差超过 2 px")
    T=np.eye(4);T[:3,:3]=cv2.Rodrigues(rv)[0];T[:3,3]=tv[:,0]
    wrist=np.linalg.inv(model.lift) @ model.pose(arm["joints"],0)[1]
    return {"frame":name,"camera_session":meta["capture_session"],"image_time":stamp,"arm_time":arm["timestamp"],
            "joints_rad":arm["joints"],"color_intrinsics":k,"board":spec,"rms_px":rms,
            "T_camera_from_marker":T.tolist(),"T_body4_from_right_link26":wrist.tolist()},data
