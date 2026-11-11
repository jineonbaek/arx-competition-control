"""Shared D405 RGB-D capture, point clouds and taught horizontal handle previews."""
import base64
import hashlib
import json
import os
import re
import sys
import threading
import time
import uuid
from pathlib import Path

import cv2
import numpy as np
from handle_preview import handle_line
from handeye_column import ColumnModel
from handeye_validation import CANDIDATE

ROOT = Path(__file__).resolve().parents[2]
LIVE = ROOT / 'web/public/camera-live'
# Keep the existing cache path so saved RGB-D snapshots remain readable.
CACHE = ROOT / 'web/hardware/runtime/grasp-preview'
HEAD_CALIBRATION = ROOT / 'vision/handeye/results/d435-cad-349622074226.json'


def roi_pixels(value, w, h):
    if not isinstance(value,list) or len(value)!=4 or any(type(v)!=int for v in value):
        raise ValueError('请在彩色图上框选电池')
    x,y,r,b=value
    if not (0<=x<r<=w and 0<=y<b<=h and r-x>=12 and b-y>=12):
        raise ValueError('选框超出图像或小于 12 像素')
    return value


def camera_intrinsics(k):
    vendor=str(ROOT/'vision/vendor-python')
    if vendor not in sys.path:sys.path.insert(0,vendor)
    import pyrealsense2 as rs
    intr=rs.intrinsics()
    intr.width,intr.height=k['width'],k['height']
    intr.fx,intr.fy,intr.ppx,intr.ppy=k['fx'],k['fy'],k['cx'],k['cy']
    intr.model=getattr(rs.distortion,k['model'].split('.')[-1])
    intr.coeffs=k['coeffs']
    return rs, intr


def xyz_image(raw, scale, k, *, max_depth_m=1.5):
    rs, intr = camera_intrinsics(k)
    if raw.dtype!=np.uint16 or raw.shape!=(intr.height,intr.width) or not np.isfinite(scale) or scale<=0:
        raise ValueError('RGB-D 深度格式或单位无效')
    z=raw.astype(np.float32)*scale
    v,u=np.nonzero((z>=.07)&(z<=max_depth_m))
    xyz=np.zeros((*raw.shape,3),np.float32)
    if len(v):
        xyz[v,u]=[rs.rs2_deproject_pixel_to_point(intr,[float(x),float(y)],float(z[y,x])) for y,x in zip(v,u)]
    return xyz


def atomic_json(path, value):
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value,allow_nan=False,separators=(',',':')))
    os.replace(tmp,path)


class GraspPreview:
    def __init__(self, controller, depth, cache=None, *, camera="d405"):
        if camera not in ("d435","d405"):raise ValueError("Unsupported RGB-D camera")
        self.camera=camera
        self._cache=Path(cache) if cache is not None else None
        self.ctrl=controller
        self.depth=depth
        self.model=None
        self.lock=threading.Lock()

    @property
    def cache(self):
        return self._cache if self._cache is not None else (CACHE if self.camera=="d405" else CACHE.parent/"grasp-head")

    def latest(self):
        path=self.cache/'view.json'
        view = json.loads(path.read_text()) if path.exists() else {}
        # A legacy capture has no exposure-time lift value for a base_link conversion.
        return view if view.get("base_from_color") and view.get("frame_id") == "base_link" else {"id": None}

    def action(self,data):
        if not isinstance(data,dict):raise ValueError('无效预览请求')
        if not self.lock.acquire(blocking=False):raise ValueError('已有采集正在运行')
        try:
            self.cache.mkdir(parents=True,exist_ok=True)
            if data.get('op')=='capture':return self.capture()
            raise ValueError('未知预览操作')
        finally:self.lock.release()

    def capture(self):
        calibration_path=CANDIDATE if self.camera=='d405' else HEAD_CALIBRATION
        calibration_blob=calibration_path.read_bytes()
        cal=json.loads(calibration_blob)
        if self.camera=='d405' and self.model is None:self.model=ColumnModel()
        try:token=self.depth.acquire_grasp(self.camera)
        except (RuntimeError,OSError) as e:raise ValueError(str(e)) from e
        try:meta,jpeg,png,q,arm_time,height,body_time=self.read_pair()
        finally:self.depth.release_handeye(token)
        k=meta['color_intrinsics'];d=meta['depth']
        if calibration_path.read_bytes()!=calibration_blob:raise ValueError('采集期间外参发生变化')
        serial=cal['camera']['serial'] if self.camera=='d405' else cal['serial']
        if k['serial']!=serial:raise ValueError('当前相机序列号与外参不一致')
        raw=cv2.imdecode(np.frombuffer(png,np.uint8),cv2.IMREAD_UNCHANGED)
        bgr=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        if raw is None or bgr is None or bgr.shape[:2]!=raw.shape:raise ValueError('RGB-D 图像尺寸不匹配')
        xyz=xyz_image(raw,d['scale_m_per_unit'],k,max_depth_m=1.5 if self.camera=='d405' else 3.)
        rgb=bgr[:,:,::-1].copy()
        if self.camera=='d405':
            T=self.model.wrist_transform(q,height)@np.asarray(cal['T_right_link26_from_D405_color'])
        else:T=np.asarray(cal['T_base_from_color_optical'],float)
        if (T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1])
                or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(T[:3,:3])<.99):
            raise ValueError('相机外参不是有效刚性变换')
        ident=uuid.uuid4().hex
        tmp=self.cache/'frame.tmp.npz'
        np.savez_compressed(tmp,id=ident,camera_id=self.camera,calibration_sha256=hashlib.sha256(calibration_blob).hexdigest(),xyz=xyz,rgb=rgb,raw_depth=raw,scale=d['scale_m_per_unit'],
                            intrinsics=json.dumps(k),base_from_color=T,lift_rad=height,body_time=body_time,joints=q,arm_time=arm_time,
                            image_time=meta['captured_at'])
        yy,xx=np.nonzero(xyz[::4,::4,2]>0);yy*=4;xx*=4
        points_base = xyz[yy,xx] @ T[:3,:3].T + T[:3,3]
        points=np.column_stack((points_base.round(4),rgb[yy,xx])).tolist()
        view=dict(id=ident,frame_id='base_link',base_from_color=T.tolist(),time=meta['captured_at'],width=k['width'],height=k['height'],points=points,
                  image='data:image/jpeg;base64,'+base64.b64encode(jpeg).decode(),result=None,
                  paired_ms=round(abs(arm_time-meta['captured_at'])*1000),roi=None)
        os.replace(tmp,self.cache/'frame.npz')
        atomic_json(self.cache/'view.json',view)
        return view

    def read_pair(self):
        start=time.time();end=time.monotonic()+16;history=[];reason='等待对齐深度帧'
        while time.monotonic()<end:
            if not self.depth.enabled(self.camera):raise ValueError('深度采集被标定接管或相机已退回彩色模式')
            s=self.ctrl.snapshot();arm=s['arms']['right'];body=s['body']
            if not self.ctrl.adapter.body_calibrated():raise ValueError('升降回零状态未就绪')
            if s['chassis_active'] or not arm.get('fresh') or not body.get('fresh'):
                raise ValueError('请保持底盘静止，并确认右臂和升降反馈在线')
            q=np.asarray(arm['joints'],float)
            if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('右臂反馈无效')
            stamp=arm['timestamp']
            if not history or stamp>history[-1][0]:history.append((stamp,q,body['height'],body['timestamp']))
            history=[v for v in history if stamp-v[0]<.75]
            try:
                meta=json.loads((LIVE/(self.camera+'.json')).read_text());d=meta.get('depth',{})
                t=meta.get('captured_at') or 0
                if (len(history)>=4 and history[-1][0]-history[0][0]>=.35 and
                    history[0][0]<=t<=history[-1][0]+.05 and t>=start and -.05<=time.time()-t<.8 and
                    meta.get('state')=='online' and d.get('state')=='online' and d.get('aligned_to_color') is True and
                    d.get('timestamp_gap_ms',1000)<=50):
                    ts,paired,height,body_time=min(history,key=lambda v:abs(v[0]-t))
                    stable=np.max(np.ptp([v[1] for v in history],axis=0))<np.radians(.25) and np.ptp([v[2] for v in history])<.05
                    if max(abs(t-ts),abs(t-body_time))<.20 and stable:
                        rgb=meta['raw_image_file'];z=d['image_file']
                        m=re.fullmatch(self.camera+r'-v-([a-f0-9]+-\d+)\.jpg',rgb)
                        if m and z==self.camera+'-z-'+m[1]+'.png':
                            return meta,(LIVE/rgb).read_bytes(),(LIVE/z).read_bytes(),paired,ts,height,body_time
                    reason='等待与静止右臂配对的 RGB-D 帧'
                elif d.get('message'):reason=d['message']
            except (FileNotFoundError,json.JSONDecodeError):pass
            time.sleep(.055)
        raise ValueError(reason)

    def handle_view(self, view=None):
        if view is None:
            view = self.latest()
        return {key:view.get(key) for key in ("id","time","image","width","height","frame_id","points","base_from_color")}

    def handle_action(self, data):
        if not isinstance(data,dict):
            raise ValueError("无效参数")
        if data.get("op") == "capture":
            return self.handle_view(self.action({"op":"capture"}))
        if data.get("op") not in ("plan","teach"):
            raise ValueError("仅支持深度预览与示教记录")
        with self.lock:
            with np.load(self.cache/"frame.npz",allow_pickle=False) as f:
                if str(f["id"]) != data.get("id") or "base_from_color" not in f:
                    raise ValueError("快照已更新，请重新读取并框选")
                xyz=f["xyz"]
                roi=roi_pixels(data.get("roi"),xyz.shape[1],xyz.shape[0])
                transform=f["base_from_color"]
                line=handle_line(xyz,transform,roi,data.get("clearance_mm"))
                rs,intr=camera_intrinsics(json.loads(str(f["intrinsics"])))
                pixels=[]
                for key in ("pre_mm","feature_mm"):
                    camera=transform[:3,:3].T @ (np.asarray(line[key])*.001-transform[:3,3])
                    if camera[2]<=0:
                        raise ValueError("预抓点位于快照相机背面，请减小预抓距离")
                    pixels.append(rs.rs2_project_point_to_pixel(intr,camera.tolist()))
                line.update(pixels=pixels,roi=roi,id=data["id"])
                height=float(f["lift_rad"])
                captured=float(f["image_time"])
            profile=self.cache.parent/"handle-teach-right.json"
            if data["op"]=="teach":
                if data.get("confirmed") is not True or not 0 <= time.time()-captured <= 900:
                    raise ValueError("请用本次示教前的新快照并确认电池、平台与底盘位置")
                samples=[]
                for _ in range(6):
                    state=self.ctrl.snapshot()
                    arm=state["arms"]["right"]
                    pose=arm.get("base_wrist")
                    if (not arm.get("fresh") or pose is None or state["chassis_active"]
                            or abs(state["body"]["height"]-height)>.05):
                        raise ValueError("示教需要底盘、升降静止，且配对右腕反馈有效")
                    samples.append((arm["timestamp"],pose,arm["joints"]))
                    time.sleep(.08)
                poses=np.asarray([item[1] for item in samples])
                joints=np.asarray([item[2] for item in samples])
                if (len({item[0] for item in samples})<4 or samples[-1][0]-samples[0][0]<.3
                        or np.max(np.ptp(poses[:,:3],axis=0))>1 or np.max(np.ptp(joints,axis=0))>np.radians(.2)):
                    raise ValueError("右腕尚未停稳，请稳定夹住后重新记录")
                taught=dict(frame="base_link",feature_mm=line["feature_mm"],wrist_pose=poses[-1].tolist(),
                            offset_mm=(poses[-1,:3]-line["feature_mm"]).tolist(),time=time.time(),
                            snapshot=data["id"],roi=roi)
                atomic_json(profile,taught)
            taught=json.loads(profile.read_text()) if profile.is_file() else None
            if taught:
                for key,src in (("wrist_pre_mm","pre_mm"),("wrist_end_mm","feature_mm")):
                    line[key]=(np.asarray(line[src])+taught["offset_mm"]).tolist()
            line["teach"]=taught
            return line
