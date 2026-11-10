"""One D405 snapshot and GPU result for read-only GraspNet visualization."""
import base64
import json
import os
import re
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import cv2
import numpy as np
from handeye_column import ColumnModel
from handeye_validation import CANDIDATE

ROOT = Path(__file__).resolve().parents[2]
LIVE = ROOT / 'web/public/camera-live'
CACHE = ROOT / 'web/hardware/runtime/grasp-preview'
DEMO = Path.home() / 'arx-graspnet-demo/grasp'


def roi_pixels(value, w, h):
    if not isinstance(value,list) or len(value)!=4 or any(type(v)!=int for v in value):
        raise ValueError('请在彩色图上框选电池')
    x,y,r,b=value
    if not (0<=x<r<=w and 0<=y<b<=h and r-x>=12 and b-y>=12):
        raise ValueError('选框超出图像或小于 12 像素')
    return value


def xyz_image(raw, scale, k):
    vendor=str(ROOT/'vision/vendor-python')
    if vendor not in sys.path:sys.path.insert(0,vendor)
    import pyrealsense2 as rs
    intr=rs.intrinsics()
    intr.width,intr.height=k['width'],k['height']
    intr.fx,intr.fy,intr.ppx,intr.ppy=k['fx'],k['fy'],k['cx'],k['cy']
    intr.model=getattr(rs.distortion,k['model'].split('.')[-1])
    intr.coeffs=k['coeffs']
    if raw.dtype!=np.uint16 or raw.shape!=(intr.height,intr.width) or not np.isfinite(scale) or scale<=0:
        raise ValueError('D405 深度格式或单位无效')
    z=raw.astype(np.float32)*scale
    v,u=np.nonzero((z>=.07)&(z<=1.5))
    xyz=np.zeros((*raw.shape,3),np.float32)
    if len(v):
        xyz[v,u]=[rs.rs2_deproject_pixel_to_point(intr,[float(x),float(y)],float(z[y,x])) for y,x in zip(v,u)]
    return xyz


def atomic_json(path, value):
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value,allow_nan=False,separators=(',',':')))
    os.replace(tmp,path)


class GraspPreview:
    def __init__(self, controller, depth):
        self.ctrl=controller
        self.depth=depth
        self.model=None
        self.lock=threading.Lock()

    def latest(self):
        path=CACHE/'view.json'
        view = json.loads(path.read_text()) if path.exists() else {}
        # A legacy capture has no exposure-time lift value for a base_link conversion.
        return view if view.get("frame_id") == "base_link" else {"id": None}

    def action(self,data):
        if not isinstance(data,dict):raise ValueError('无效预览请求')
        if not self.lock.acquire(blocking=False):raise ValueError('已有采集或推理正在运行')
        try:
            CACHE.mkdir(parents=True,exist_ok=True)
            if data.get('op')=='capture':return self.capture()
            if data.get('op')=='infer':return self.infer(data)
            raise ValueError('未知预览操作')
        finally:self.lock.release()

    def capture(self):
        if self.model is None:self.model=ColumnModel()
        try:token=self.depth.acquire_grasp()
        except (RuntimeError,OSError) as e:raise ValueError(str(e)) from e
        try:meta,jpeg,png,q,arm_time,height,body_time=self.read_pair()
        finally:self.depth.release_handeye(token)
        k=meta['color_intrinsics'];d=meta['depth']
        cal=json.loads(CANDIDATE.read_text())
        if k['serial']!=cal['camera']['serial']:raise ValueError('当前 D405 序列号与手眼外参不一致')
        raw=cv2.imdecode(np.frombuffer(png,np.uint8),cv2.IMREAD_UNCHANGED)
        bgr=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        if raw is None or bgr is None or bgr.shape[:2]!=raw.shape:raise ValueError('RGB-D 图像尺寸不匹配')
        xyz=xyz_image(raw,d['scale_m_per_unit'],k)
        rgb=bgr[:,:,::-1].copy()
        G=self.model.wrist_transform(q,height)
        T=G@np.asarray(cal['T_right_link26_from_D405_color'])
        ident=uuid.uuid4().hex
        tmp=CACHE/'frame.tmp.npz'
        np.savez_compressed(tmp,id=ident,xyz=xyz,rgb=rgb,raw_depth=raw,scale=d['scale_m_per_unit'],
                            intrinsics=json.dumps(k),base_from_color=T,lift_rad=height,body_time=body_time,joints=q,arm_time=arm_time,
                            image_time=meta['captured_at'])
        yy,xx=np.nonzero(xyz[::4,::4,2]>0);yy*=4;xx*=4
        points=np.column_stack((xyz[yy,xx].round(4),rgb[yy,xx])).tolist()
        view=dict(id=ident,frame_id='base_link',time=meta['captured_at'],width=k['width'],height=k['height'],points=points,
                  image='data:image/jpeg;base64,'+base64.b64encode(jpeg).decode(),result=None,
                  paired_ms=round(abs(arm_time-meta['captured_at'])*1000),roi=None)
        os.replace(tmp,CACHE/'frame.npz')
        atomic_json(CACHE/'view.json',view)
        return view

    def read_pair(self):
        start=time.time();end=time.monotonic()+16;history=[];reason='等待对齐深度帧'
        while time.monotonic()<end:
            if not self.depth.enabled('d405'):raise ValueError('深度采集被标定接管或相机已退回彩色模式')
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
                meta=json.loads((LIVE/'d405.json').read_text());d=meta.get('depth',{})
                t=meta.get('captured_at') or 0
                if (len(history)>=4 and history[-1][0]-history[0][0]>=.35 and
                    history[0][0]<=t<=history[-1][0]+.05 and t>=start and -.05<=time.time()-t<.8 and
                    meta.get('state')=='online' and d.get('state')=='online' and d.get('aligned_to_color') is True and
                    d.get('timestamp_gap_ms',1000)<=50):
                    ts,paired,height,body_time=min(history,key=lambda v:abs(v[0]-t))
                    stable=np.max(np.ptp([v[1] for v in history],axis=0))<np.radians(.25) and np.ptp([v[2] for v in history])<.05
                    if max(abs(t-ts),abs(t-body_time))<.20 and stable:
                        rgb=meta['raw_image_file'];z=d['image_file']
                        m=re.fullmatch(r'd405-v-([a-f0-9]+-\d+)\.jpg',rgb)
                        if m and z=='d405-z-'+m[1]+'.png':
                            return meta,(LIVE/rgb).read_bytes(),(LIVE/z).read_bytes(),paired,ts,height,body_time
                    reason='等待与静止右臂配对的 RGB-D 帧'
                elif d.get('message'):reason=d['message']
            except (FileNotFoundError,json.JSONDecodeError):pass
            time.sleep(.055)
        raise ValueError(reason)

    def infer(self,data):
        view=self.latest()
        if not view.get('id') or view['id']!=data.get('id'):raise ValueError('快照已更新，请重新框选')
        roi=roi_pixels(data.get('roi'),view['width'],view['height'])
        executable=DEMO/'.venv/bin/python'
        if not executable.is_file():raise ValueError('ROG GraspNet 环境缺失')
        output=CACHE/'inference.json'
        env=dict(os.environ,OMP_NUM_THREADS='4',OPENBLAS_NUM_THREADS='4',MKL_NUM_THREADS='4')
        try:
            run=subprocess.run([str(executable),str(ROOT/'vision/grasp/infer_d405.py'),str(DEMO),
                                str(CACHE/'frame.npz'),view['id'],str(output),*map(str,roi)],
                               env=env,capture_output=True,text=True,timeout=90)
        except subprocess.TimeoutExpired as e:raise ValueError('GraspNet 推理超时') from e
        if run.returncode:
            msg=(run.stderr.strip() or run.stdout.strip()).splitlines()
            raise ValueError('GraspNet 推理失败：'+(msg[-1][:300] if msg else '进程异常退出'))
        result=json.loads(output.read_text())
        if result['id']!=view['id'] or result['roi']!=roi:raise ValueError('推理结果与快照不匹配')
        view.update(result=result,roi=roi)
        atomic_json(CACHE/'view.json',view)
        return view
