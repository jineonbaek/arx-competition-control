"""Astra Pro native depth; the separate UVC color stream is NOT registered."""
import ctypes
import math
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]


def error_message(error):
    try:return str(error)
    except Exception:
        return "; ".join(x.decode(errors="replace") for x in error.args if isinstance(x,bytes)) or type(error).__name__


class AstraDepth:
    def __init__(self):
        sys.path.insert(0,str(ROOT/"vision/vendor-python"))
        from openni import openni2,_openni2 as api
        # Must match Orbbec's OniCTypes.h BEFORE any device info is copied by C.
        if ctypes.sizeof(api.OniDeviceInfo)!=1028:
            raise RuntimeError("Unsafe OpenNI ABI: run vision/scripts/setup-depth.py")
        self.sdk=openni2;self.device=None;self.stream=None
        openni2.initialize(str(ROOT/"vision/vendor-openni"))
        try:
            devices=[uri for uri in openni2.Device.enumerate_uris() if uri.startswith(b"2bc5/0403@")]
            if len(devices)!=1:raise RuntimeError("Expected exactly one Astra Pro depth device")
            self.device=openni2.Device(devices[0])
            self.serial=self.device.get_device_info().serialNumber.decode(errors="replace")
            self.stream=self.device.create_depth_stream()
            self.stream.set_video_mode(api.OniVideoMode(pixelFormat=api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM,resolutionX=640,resolutionY=480,fps=30))
            self.stream.set_mirroring_enabled(False)
            self.intrinsics=dict(fx=640/(2*math.tan(self.stream.get_horizontal_fov()/2)),fy=480/(2*math.tan(self.stream.get_vertical_fov()/2)),cx=319.5,cy=239.5,source="SDK_FOV_derived_center_assumed",distortion_model="unknown")
            self.stream.start()
        except Exception:
            self.close();raise

    def read(self):
        self.sdk.wait_for_any_stream([self.stream],2000)
        frame=self.stream.read_frame()
        raw=np.frombuffer(frame.get_buffer_as_uint16(),dtype=np.uint16).reshape(frame.height,frame.width).copy()
        return raw,dict(frame_number=frame.frameIndex,sensor_timestamp_ms=frame.timestamp/1000.,captured_at=time.time(),camera_serial=self.serial,age_source="host_receipt_only",timestamp_domain="device_clock",pairing="independent_unsynchronized",registration="native_depth_unregistered")

    def close(self):
        try:
            if self.stream is not None:self.stream.close()
        finally:
            try:
                if self.device is not None:self.device.close()
            finally:self.sdk.unload()
