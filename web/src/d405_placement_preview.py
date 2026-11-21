"""Explicit read-only D405 capture and partial calibration-line preview."""
import json
import threading
from pathlib import Path
import numpy as np
from d405_left_edges import measure_left_edges
from d405_alignment_overlay import write_horizontal_overlay
from d405_battery_edges import detect_face,select_depth_preview


class D405PlacementPreview:
    def __init__(self,controller,capture,root):
        self.ctrl=controller;self.capture=capture;self.root=Path(root);self.lock=threading.Lock()

    def action(self,data):
        if data!={'op':'refresh'}:raise ValueError('仅支持刷新 D405 标定快照')
        if not self.lock.acquire(blocking=False):raise ValueError('D405 正在采集标定快照')
        try:
            start=self.ctrl.snapshot();frames=[];observations=[];seen=set()
            for _ in range(3):
                view=self.capture.action({'op':'capture'})
                with np.load(self.capture.cache/'frame.npz',allow_pickle=False) as stored:
                    frame={k:stored[k] for k in stored.files}
                if str(frame['id'])!=view['id']:raise ValueError('D405 快照已被另一轮采样替换，请重试')
                if view['id'] in seen:raise ValueError('D405 重复曝光不能构成四棱稳定证据')
                seen.add(view['id'])
                current=self.ctrl.snapshot()
                if (current['command_epoch']!=start['command_epoch'] or current.get('chassis_active')
                    or abs(current['body']['height']-start['body']['height'])>.02
                    or np.max(np.abs(np.asarray(current['arms']['right']['joints'])-start['arms']['right']['joints']))>np.radians(.25)):
                    raise ValueError('多帧标定期间机械臂或底盘发生变化，请停稳后刷新')
                frames.append(frame);observations.append(detect_face(frame['rgb'],xyz=frame['xyz'],base_from_color=frame['base_from_color']))
                if observations[-1] is not None:break
            consensus=select_depth_preview(observations)
            frame=frames[consensus[0] if consensus else -1]
            frame['battery_face_observation']=consensus[1] if consensus else False
            result=dict(status='unavailable',frame_id=str(frame['id']),image_time=float(frame['image_time']),
                calibration_sha256=str(frame['calibration_sha256']))
            try:
                tcp=np.asarray(json.loads((self.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
                tool=self.ctrl.models['right'].wrist_transform(frame['joints'],float(frame['lift_rad']))@tcp
                result=measure_left_edges(frame,tool,self.root)
            except (OSError,ValueError) as error:result['reason']=str(error)
            return write_horizontal_overlay(frame,result,self.root/'web/public/camera-overlay')
        finally:self.lock.release()
