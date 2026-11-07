"""Astra depth with validated factory or measured color registration."""
import ctypes
import math
from pathlib import Path
import sys
import time
import numpy as np
from astra_registration import factory_parameters,load_calibration,Registration,intrinsics

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
            if not self.serial:
                self.serial=self.device.get_property(3,ctypes.c_char*256).value.decode(errors="replace")
            self.stream=self.device.create_depth_stream()
            self.stream.set_video_mode(api.OniVideoMode(pixelFormat=api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM,resolutionX=640,resolutionY=480,fps=30))
            self.stream.set_mirroring_enabled(False)
            self.intrinsics=dict(fx=640/(2*math.tan(self.stream.get_horizontal_fov()/2)),fy=480/(2*math.tan(self.stream.get_vertical_fov()/2)),cx=319.5,cy=239.5,source="SDK_FOV_derived_center_assumed",distortion_model="unknown")
            self.configure_registration(api)
            self.stream.start()
        except Exception:
            self.close();raise

    def configure_registration(self,api):
        self.projector=None;self.aligned_to_color=False;self.registration="native_depth_unregistered"
        modes=api.OniImageRegistrationMode
        self.device.set_image_registration_mode(modes.ONI_IMAGE_REGISTRATION_OFF)
        if self.device.get_image_registration_mode().value!=0:
            raise RuntimeError("Cannot confirm native Astra depth coordinates")
        factory=factory_parameters(self.device)
        self.calibration=dict(status="required",factory=factory,message="Astra RGB-D calibration required")
        if not self.serial or not all(c.isascii() and (c.isalnum() or c in "-_") for c in self.serial):
            raise ValueError("Invalid Astra serial number for calibration lookup")
        path=ROOT/"vision/config"/("astra-"+self.serial+".json")
        try:
            if path.is_file():
                data=load_calibration(path,self.serial)
                self.projector=Registration(data);self.intrinsics=self.projector.intrinsics
                self.registration="measured_depth_to_color"
                self.calibration=dict(status="loaded",source="measured_rgbd_calibration",sha256=data['sha256'],validation="pending_physical_check")
            elif factory['status']=="valid":
                self.device.set_image_registration_mode(modes.ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR)
                if self.device.get_image_registration_mode().value!=1:
                    raise RuntimeError("Astra SDK did not confirm depth-to-color mode")
                params=factory['parameters'];fx,fy,cx,cy=params['r_intr_p']
                k=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
                # Match the pinned Orbbec ROS camera_info coefficient order.
                d=[params['r_k'][i] for i in [0,1,3,4,2]]
                self.intrinsics=intrinsics(k,d,"factory_rgb_calibration")
                self.registration="sdk_depth_to_color"
                self.calibration=dict(status="loaded",source="Orbbec_OpenNI_property_14",sha256=factory['sha256'],validation="pending_physical_check")
            else:return
            self.aligned_to_color=True
        except Exception as error:
            self.device.set_image_registration_mode(modes.ONI_IMAGE_REGISTRATION_OFF)
            if self.device.get_image_registration_mode().value!=0:
                raise RuntimeError("Cannot restore native Astra depth coordinates") from error
            self.projector=None;self.registration="native_depth_unregistered"
            self.calibration.update(status="error",message=error_message(error))

    def read(self):
        self.sdk.wait_for_any_stream([self.stream],2000)
        frame=self.stream.read_frame()
        raw=np.frombuffer(frame.get_buffer_as_uint16(),dtype=np.uint16).reshape(frame.height,frame.width).copy()
        if self.projector is not None:raw=self.projector.apply(raw)
        return raw,dict(frame_number=frame.frameIndex,sensor_timestamp_ms=frame.timestamp/1000.,captured_at=time.time(),camera_serial=self.serial,age_source="host_receipt_only",timestamp_domain="device_clock",pairing="independent_unsynchronized",registration=self.registration,aligned_to_color=self.aligned_to_color,calibration=self.calibration)

    def close(self):
        try:
            if self.stream is not None:self.stream.close()
        finally:
            try:
                if self.device is not None:self.device.close()
            finally:self.sdk.unload()
