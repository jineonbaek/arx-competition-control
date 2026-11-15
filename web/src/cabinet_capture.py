"""Read existing color files + feedback only. Never acquires depth or motor ownership."""
import hashlib
import json
from pathlib import Path
import re
import time
import uuid
import cv2
import numpy as np
from agent.cabinet_rgb_geometry import camera_matrix
from agent.placement_geometry import rigid

ROOT=Path(__file__).resolve().parents[2]
CALIBRATIONS={'d435':'vision/handeye/results/d435-cad-349622074226.json',
              'd405':'vision/handeye/results/d405-right-cc200-ee0ff2fa49e9e56285d271d6.json'}


class CabinetColorCapture:
    def __init__(self,controller,*,camera,root=ROOT):
        if camera not in CALIBRATIONS:raise ValueError('未知仓口相机')
        self.controller=controller;self.camera=camera;self.root=Path(root)

    def build_frame(self,meta,state,*,now=None,token=None):
        now=time.time() if now is None else now
        try:
            arm=state['arms']['right'];body=state['body'];stamp=float(meta['captured_at'])
            q=np.asarray(arm['joints'],float);lift=float(body['height'])
            owned=token is not None and self.controller.motion_valid(token)
            if ((state.get('motion_reserved') is not False and not owned) or state.get('chassis_active') is not False
                    or not arm.get('fresh') or not body.get('fresh') or q.shape!=(6,)
                    or not np.isfinite(q).all() or not np.isfinite([lift,stamp,now,arm['timestamp'],body['timestamp'],arm['gripper']]).all()
                    or meta['state']!='online' or not -.05<=now-stamp<=.8
                    or max(abs(stamp-arm['timestamp']),abs(stamp-body['timestamp']))>.2):
                raise ValueError('彩色画面或机器人反馈过期、未配对或正在运动')
            name=meta['raw_image_file']
            if not isinstance(name,str) or not re.fullmatch(self.camera+r'-v-[a-f0-9]+-\d+\.jpg',name):
                raise ValueError('彩色快照文件身份无效')
            calpath=self.root/CALIBRATIONS[self.camera];blob=calpath.read_bytes();cal=json.loads(blob)
            serial=cal['serial'] if self.camera=='d435' else cal['camera']['serial']
            k=meta['color_intrinsics']
            if k.get('serial')!=serial:raise ValueError('相机序列号与外参不符')
            bgr=cv2.imdecode(np.frombuffer((self.root/'web/public/camera-live'/name).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
            if bgr is None:raise ValueError('彩色图像无法解码')
            camera_matrix(k,bgr.shape)
            if self.camera=='d435':T=rigid(cal['T_base_from_color_optical'])
            else:T=rigid(self.controller.models['right'].wrist_transform(q,lift)@np.array(cal['T_right_link26_from_D405_color']))
            if calpath.read_bytes()!=blob:raise ValueError('读取期间相机标定发生变化')
            frame=dict(id=uuid.uuid4().hex,camera_id=self.camera,camera_serial=serial,rgb=bgr[:,:,::-1].copy(),
                intrinsics=json.dumps(k),calibration_sha256=hashlib.sha256(blob).hexdigest(),
                calibration_status=cal.get('status','未声明验证状态'),base_from_color=T.copy(),
                joints=q.copy(),right_gripper=float(arm['gripper']),lift_rad=lift,image_time=stamp,
                arm_time=float(arm['timestamp']),body_time=float(body['timestamp']),command_epoch=state['command_epoch'],
                body_configuration=json.dumps({key:body.get(key) for key in ('waist','head_yaw','head_pitch')}))
            left=state['arms'].get('left',{})
            if left.get('fresh'):
                frame.update(left_joints=np.array(left['joints'],float),left_arm_time=float(left['timestamp']))
            return frame
        except (KeyError,TypeError,json.JSONDecodeError,OSError) as e:raise ValueError('仓口彩色帧或标定不可用：'+str(e)) from e

    def capture(self,*,token=None):
        deadline=time.monotonic()+3.;samples=[];reason='等待静止反馈与新彩色帧'
        while time.monotonic()<deadline:
            try:
                state=self.controller.snapshot();meta=json.loads((self.root/'web/public/camera-live'/(self.camera+'.json')).read_text())
                frame=self.build_frame(meta,state,token=token)
                ts=frame['arm_time']
                if not samples or ts>samples[-1][0]:samples.append((ts,frame['joints'],frame['lift_rad']))
                samples=[s for s in samples if ts-s[0]<.75]
                if (len(samples)>=4 and samples[-1][0]-samples[0][0]>=.35
                        and np.max(np.ptp([s[1] for s in samples],axis=0))<np.radians(.25)
                        and np.ptp([s[2] for s in samples])<.05):return frame
            except (ValueError,OSError,json.JSONDecodeError) as e:reason=str(e)
            time.sleep(.05)
        raise ValueError(reason)
