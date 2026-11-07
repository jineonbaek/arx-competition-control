"""Stationary-target RGB/IR snapshots through the existing camera owner only."""
import ctypes
import json
import os
from pathlib import Path
import re
import sys
import time
import cv2
import numpy as np
from astra_depth import error_message
from astra_registration import factory_parameters

ROOT=Path(__file__).resolve().parents[2]


def capture_pair(color,output):
    sys.path.insert(0,str(ROOT/'vision/vendor-python'))
    from openni import openni2 as sdk,_openni2 as api
    if ctypes.sizeof(api.OniDeviceInfo)!=1028:raise RuntimeError('Unsafe OpenNI ABI')
    device=None;stream=None;frame=None
    sdk.initialize(str(ROOT/'vision/vendor-openni'))
    try:
        uris=[u for u in sdk.Device.enumerate_uris() if u.startswith(b'2bc5/0403@')]
        if len(uris)!=1:raise RuntimeError('Expected one Astra Pro depth/IR device')
        device=sdk.Device(uris[0]);serial=device.get_property(3,ctypes.c_char*256).value.decode()
        stream=device.create_ir_stream()
        if stream is None:raise RuntimeError('Astra infrared stream unavailable')
        modes=stream.get_sensor_info().videoModes
        gray16=api.OniPixelFormat.ONI_PIXEL_FORMAT_GRAY16
        modes=[m for m in modes if m.resolutionX==640 and m.resolutionY==480 and m.fps==30
               and getattr(m.pixelFormat,'value',m.pixelFormat)==gray16]
        if not modes:raise RuntimeError('No native 640x480 30fps GRAY16 infrared profile')
        stream.set_video_mode(modes[0]);stream.set_mirroring_enabled(False);stream.start()
        started=time.time()
        for _ in range(8):
            sdk.wait_for_any_stream([stream],2000);frame=stream.read_frame()
        infrared=np.frombuffer(frame.get_buffer_as_uint16(),np.uint16).reshape(frame.height,frame.width).copy()
        ir_received=time.time()
        # RGB uses the owner's existing UVC handle; flush queued frames after IR warm-up.
        for _ in range(4):
            ok,rgb=color.read()
            if not ok or rgb is None:raise RuntimeError('No RGB frame during infrared snapshot')
        color_received=time.time()
        if rgb.shape[:2]!=(480,640):raise RuntimeError('Unexpected Astra color profile')
        output.mkdir(parents=True,exist_ok=False)
        for name,im in [('color.png',rgb),('infrared.png',infrared),
                        ('infrared-preview.jpg',cv2.normalize(infrared,None,0,255,cv2.NORM_MINMAX,cv2.CV_8U))]:
            if not cv2.imwrite(str(output/name),im):raise RuntimeError('Could not save '+name)
        return dict(serial=serial,depth_active=False,pairing='stationary_target_only_no_hardware_sync',
                    started_at=started,ir_received_at=ir_received,color_received_at=color_received,
                    host_receipt_gap_ms=(color_received-ir_received)*1000,
                    infrared=dict(width=frame.width,height=frame.height,encoding='mono16_intensity',mirrored=False,
                                  frame_number=frame.frameIndex,sensor_timestamp_ms=frame.timestamp/1000),
                    factory=factory_parameters(device),directory=str(output))
    finally:
        frame=None
        try:
            if stream is not None:stream.close()
        finally:
            try:
                if device is not None:device.close()
            finally:sdk.unload()


class CalibrationCapture:
    def __init__(self,directory,session):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        self.request=self.directory/'request.json';self.result=self.directory/'result.json'
        self.session=session;self.last=None

    def poll(self,color,depth_busy):
        try:request=json.loads(self.request.read_text())
        except (OSError,ValueError):return False
        if not isinstance(request,dict):return False
        token=request.get('token')
        if not isinstance(token,str) or not re.fullmatch('[a-f0-9]{32}',token):return False
        if request.get('session')!=self.session or token==self.last:return False
        self.last=token
        result=dict(token=token,session=self.session,captured_at=time.time())
        try:
            if depth_busy:raise RuntimeError('Disable Astra depth before calibration capture')
            result.update(capture_pair(color,self.directory/token),state='captured')
        except Exception as error:result.update(state='error',message=error_message(error))
        temp=self.result.with_suffix('.tmp');temp.write_text(json.dumps(result,allow_nan=False));os.replace(temp,self.result)
        return True
