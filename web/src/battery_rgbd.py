"""D435 RGB-D capture + experimental handle overlay. Camera I/O only."""
from pathlib import Path
import collections
import json
import sys
import time
import uuid
import cv2
import numpy as np
from battery_vision import detect_handle,locate_handle,frames_usable,draw_result

ROOT=Path(__file__).resolve().parents[2]

def run(spec,stop,publish,out):
    sys.path.insert(0,str(ROOT/'vision/vendor-python'))
    import pyrealsense2 as rs
    sequence=0;session=uuid.uuid4().hex[:10];recent=collections.deque()
    while not stop.is_set():
        pipeline=None;started=False
        try:
            publish(out,'d435',None,sequence,None,'connecting','正在连接同步彩色与深度')
            devices=[d for d in rs.context().query_devices() if d.get_info(rs.camera_info.name)=='Intel RealSense D435' or d.get_info(rs.camera_info.name)=='RealSense D435']
            if len(devices)!=1:raise RuntimeError('未找到唯一的 D435')
            serial=devices[0].get_info(rs.camera_info.serial_number)
            pipeline=rs.pipeline();config=rs.config();config.enable_device(serial)
            config.enable_stream(rs.stream.depth,640,480,rs.format.z16,15)
            config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,15)
            profile=pipeline.start(config);started=True
            scale=profile.get_device().first_depth_sensor().get_depth_scale()
            align=rs.align(rs.stream.color);warmup=15;last=0
            for pattern in ['d435-v-*.jpg','d435-r-*.jpg']:
                for old in out.glob(pattern):
                    if time.time()-old.stat().st_mtime>60:old.unlink(missing_ok=True)
            print('D435 RGB-D started',serial,'depth_scale',scale,'SDK',rs.__version__,flush=True)
            while not stop.is_set():
                frames=pipeline.wait_for_frames(4000);received_at=time.time();received_mono=time.monotonic()
                if warmup:warmup-=1;continue
                if time.monotonic()-last<.2:continue
                original_color=frames.get_color_frame();original_depth=frames.get_depth_frame()
                domain=original_color.get_frame_timestamp_domain()
                epoch_clock=domain in (rs.timestamp_domain.system_time,rs.timestamp_domain.global_time)
                sensor_age=received_at-original_color.get_timestamp()/1000 if epoch_clock else 0.0
                captured_at=original_color.get_timestamp()/1000 if epoch_clock else received_at
                aligned=align.process(frames);color=aligned.get_color_frame();depth=aligned.get_depth_frame()
                if not color or not depth:raise RuntimeError('同步彩色或深度帧缺失')
                image=np.asanyarray(color.get_data());depth_m=np.asanyarray(depth.get_data()).astype(np.float32)*scale
                intrinsic=depth.profile.as_video_stream_profile().intrinsics
                intr=dict(fx=intrinsic.fx,fy=intrinsic.fy,cx=intrinsic.ppx,cy=intrinsic.ppy)
                intr['ray_fn']=lambda pixels: [rs.rs2_deproject_pixel_to_point(intrinsic,[float(p[0]),float(p[1])],1.) for p in pixels]
                detection=detect_handle(image)
                synced=frames_usable(original_color.get_timestamp(),original_depth.get_timestamp(),scale,sensor_age+time.monotonic()-received_mono)
                location=locate_handle(detection,image,depth_m,intr) if synced else dict(status='depth_invalid',bar_surface_xyz_m=None,robot_xyz_m=None,reason='RGB/depth timing invalid')
                if detection and location['status']=='surface_valid':
                    span=detection['long_px']*location['depth_median_m']/intr['fx']
                    location['approx_hole_span_m']=span
                    if not .020<=span<=.065:
                        location.update(status='depth_invalid',bar_surface_xyz_m=None,reason='Slot size inconsistent with battery model')
                if sensor_age+time.monotonic()-received_mono>.35:
                    location.update(status='depth_invalid',bar_surface_xyz_m=None,reason='Image processing exceeded freshness limit')
                rendered=draw_result(image,detection,location)
                ok,jpeg=cv2.imencode('.jpg',rendered,[cv2.IMWRITE_JPEG_QUALITY,86])
                if not ok:raise RuntimeError('识别画面编码失败')
                sequence+=1;filename=f'd435-v-{session}-{sequence}.jpg'
                p=out/filename;p.write_bytes(jpeg.tobytes())
                raw_name=f'd435-r-{session}-{sequence}.jpg';raw_path=out/raw_name
                raw_ok,raw_jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,92])
                if not raw_ok:raise RuntimeError('原始彩色帧编码失败')
                raw_path.write_bytes(raw_jpeg.tobytes());recent.append((p,raw_path))
                measurement={'detection':detection,'location':location,'camera_serial':serial,'color_frame_number':original_color.get_frame_number(),'depth_frame_number':original_depth.get_frame_number(),'color_timestamp_ms':original_color.get_timestamp(),'depth_timestamp_ms':original_depth.get_timestamp(),'timestamp_gap_ms':abs(original_color.get_timestamp()-original_depth.get_timestamp()),'depth_scale_m':scale,'intrinsics':dict(fx=intrinsic.fx,fy=intrinsic.fy,cx=intrinsic.ppx,cy=intrinsic.ppy,distortion_model=str(intrinsic.model),coeffs=list(intrinsic.coeffs)),'read_only':True,'timestamp_domain':str(domain),'frame_age_at_receipt_ms':sensor_age*1000,'age_source':'sensor_epoch_clock' if epoch_clock else 'host_receipt_only'}
                publish(out,'d435',jpeg.tobytes(),sequence,captured_at,'online',extra={'image_file':filename,'raw_image_file':raw_name,'battery':measurement})
                while len(recent)>35:
                    for expired in recent.popleft():expired.unlink(missing_ok=True)
                last=time.monotonic()
        except Exception as error:
            print('D435 RGB-D retry:',repr(error),flush=True)
            publish(out,'d435',None,sequence,None,'offline','彩色深度采集失败：'+str(error))
        finally:
            if pipeline is not None and started:
                try:pipeline.stop()
                except Exception:pass
        stop.wait(3)
    publish(out,'d435',None,sequence,None,'offline','视频采集已停止')
