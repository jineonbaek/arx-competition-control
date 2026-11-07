"""RealSense color preview with explicitly requested depth; no recognition."""
from pathlib import Path
import collections
import sys
import time
import uuid
import cv2
import numpy as np
from camera_depth import write_depth
from camera_depth_control import DepthControl

ROOT=Path(__file__).resolve().parents[2]


def usb_camera_present(product,sysfs=Path('/sys/bus/usb/devices')):
    for device in sysfs.iterdir():
        try:
            if (device/'product').read_text().strip()==product:return True
        except OSError:continue
    return False


def configure_streams(rs,config,enabled):
    config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,15)
    if enabled:config.enable_stream(rs.stream.depth,640,480,rs.format.z16,15)


def run(spec,stop,publish,out,control=None):
    control=control or DepthControl()
    sys.path.insert(0,str(ROOT/'vision/vendor-python'))
    import pyrealsense2 as rs
    key=spec['id'];model=spec['name'];sequence=0;session=uuid.uuid4().hex[:10];recent=collections.deque();last_error=''
    while not stop.is_set():
        pipeline=None;started=False;changed=False;enabled=control.enabled(key)
        try:
            if not usb_camera_present(spec['product']):raise RuntimeError('USB device not enumerated: '+model)
            publish(out,key,None,sequence,None,'connecting','Connecting color stream',extra={'depth':dict(requested=enabled,state='starting' if enabled else 'disabled',message=last_error)})
            context=rs.context()
            devices=[d for d in context.query_devices() if d.get_info(rs.camera_info.name).removeprefix('Intel ')==model]
            if len(devices)!=1:raise RuntimeError('Expected exactly one '+model)
            serial=devices[0].get_info(rs.camera_info.serial_number)
            pipeline=rs.pipeline(context);config=rs.config();config.enable_device(serial)
            configure_streams(rs,config,enabled)
            profile=pipeline.start(config);started=True
            streams=[str(p.stream_type()) for p in profile.get_streams()]
            scale=profile.get_device().first_depth_sensor().get_depth_scale() if enabled else None
            align=rs.align(rs.stream.color) if enabled else None
            warmup=15;last=0
            for pattern in [f'{key}-v-*.jpg',f'{key}-r-*.jpg',f'{key}-z-*.png',f'{key}-z-*.jpg']:
                for old in out.glob(pattern):
                    if time.time()-old.stat().st_mtime>60:old.unlink(missing_ok=True)
            print(model,'started','depth',enabled,'streams',streams,flush=True)
            while not stop.is_set():
                if control.enabled(key)!=enabled:changed=True;break
                frames=pipeline.wait_for_frames(4000);received=time.time()
                if warmup:warmup-=1;continue
                if time.monotonic()-last<.2:continue
                original_color=frames.get_color_frame()
                domain=original_color.get_frame_timestamp_domain()
                epoch_clock=domain in (rs.timestamp_domain.system_time,rs.timestamp_domain.global_time)
                captured=original_color.get_timestamp()/1000 if epoch_clock else received
                color=original_color;depth_meta=dict(requested=False,state='disabled',message=last_error);depth_paths=()
                sequence+=1;token=f'{session}-{sequence}'
                if enabled:
                    original_depth=frames.get_depth_frame();aligned=align.process(frames)
                    color=aligned.get_color_frame();depth=aligned.get_depth_frame()
                    if not color or not depth:raise RuntimeError('Missing color or depth frame')
                    intrinsic=depth.profile.as_video_stream_profile().intrinsics
                    intr=dict(fx=intrinsic.fx,fy=intrinsic.fy,cx=intrinsic.ppx,cy=intrinsic.ppy,distortion_model=str(intrinsic.model),coeffs=list(intrinsic.coeffs))
                    raw=np.asanyarray(depth.get_data())
                    depth_meta,depth_paths=write_depth(out,key,token,raw,scale,intr,True,original_depth.get_frame_number(),original_depth.get_timestamp())
                    depth_meta.update(requested=True,camera_serial=serial,captured_at=captured,timestamp_domain=str(original_depth.get_frame_timestamp_domain()),timestamp_gap_ms=abs(original_color.get_timestamp()-original_depth.get_timestamp()),age_source='sensor_epoch_clock' if epoch_clock else 'host_receipt_only')
                    last_error=''
                image=np.asanyarray(color.get_data());ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,86])
                if not ok:raise RuntimeError('Color encoding failed')
                name=f'{key}-v-{token}.jpg';path=out/name;path.write_bytes(jpeg.tobytes())
                recent.append((path,*depth_paths))
                publish(out,key,jpeg.tobytes(),sequence,captured,'online',extra={'image_file':name,'raw_image_file':name,'depth':depth_meta,'active_streams':streams})
                while len(recent)>35:
                    for old in recent.popleft():old.unlink(missing_ok=True)
                last=time.monotonic()
        except Exception as error:
            last_error=str(error)
            if enabled:
                control.request({'camera':key,'enabled':False});changed=True
                last_error='Depth failed and was disabled: '+last_error
            publish(out,key,None,sequence,None,'offline',last_error,extra={'depth':dict(requested=False,state='error',message=last_error)})
        finally:
            if pipeline is not None and started:
                try:pipeline.stop()
                except Exception as error:
                    publish(out,key,None,sequence,None,'offline','Stream stop failed: '+str(error),extra={'depth':dict(requested=control.enabled(key),state='error',message='Could not confirm stream stopped')})
                    return
        stop.wait(.05 if changed else 10)
    publish(out,key,None,sequence,None,'offline','Capture stopped',extra={'depth':dict(requested=False,state='disabled')})
