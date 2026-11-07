#!/usr/bin/env python3
"""Camera-only latest JPEG publisher. No ROS, robot SDK or CAN access."""
import argparse
import fcntl
import collections
import uuid
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from camera_depth_control import DepthControl

WEB=Path(__file__).resolve().parents[1]
OUT=WEB/'public'/'camera-live'
SESSION=uuid.uuid4().hex[:10]
CAPTURE_SESSION=None
RECENT={key:collections.deque() for key in ['d405','astra']}
SPECS=[
 {'id':'d435','name':'RealSense D435','product':'Intel(R) RealSense(TM) Depth Camera 435','index':0,'interface_suffix':':1.3','fourcc':'YUYV'},
 {'id':'d405','name':'RealSense D405','product':'Intel(R) RealSense(TM) Depth Camera 405','index':4,'interface_suffix':':1.0','fourcc':'YUYV'},
 {'id':'astra','name':'Astra Pro','product':'Astra Pro HD Camera','index':0,'interface_suffix':':1.0','fourcc':'MJPG'},
]

def select_device(spec,records):
    matches=[r['device'] for r in records if r['product']==spec['product'] and r['index']==spec['index'] and r['interface'].endswith(spec['interface_suffix'])]
    return matches[0] if len(matches)==1 else None

def devices():
    result=[]
    for p in Path('/sys/class/video4linux').glob('video*'):
        try:
            interface=(p/'device').resolve()
            product=next((q/'product').read_text().strip() for q in [interface,*interface.parents] if (q/'product').exists())
            result.append(dict(device='/dev/'+p.name,product=product,index=int((p/'index').read_text()),interface=interface.name))
        except (OSError,ValueError,StopIteration):
            continue
    return result

def atomic(path,data):
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_bytes(data)
    os.replace(tmp,path)

def publish(directory,key,jpeg,sequence,captured_at,state,message='',extra=None):
    directory.mkdir(parents=True,exist_ok=True)
    image=directory/(key+'.jpg')
    if jpeg is not None:
        atomic(image,jpeg)
    else:
        image.unlink(missing_ok=True)
    metadata=dict(state=state,sequence=sequence,captured_at=captured_at,message=message,updated_at=time.time(),capture_session=CAPTURE_SESSION,recognition_enabled=False)
    if jpeg is not None and key in RECENT and not (extra or {}).get('image_file'):
        name=f'{key}-v-{SESSION}-{sequence}.jpg';p=directory/name
        atomic(p,jpeg);metadata['image_file']=name;RECENT[key].append(p)
        while len(RECENT[key])>35:RECENT[key].popleft().unlink(missing_ok=True)
    if extra:metadata.update(extra)
    atomic(directory/(key+'.json'),json.dumps(metadata,allow_nan=False).encode())

def worker(spec,stop,control=None):
    control=control or DepthControl()
    if spec['id'] in ('d435','d405'):
        from battery_rgbd import run
        return run(spec,stop,publish,OUT,control)
    import cv2
    from astra_depth import AstraDepth,error_message
    from camera_depth import write_depth
    from astra_calibration import CalibrationCapture
    calibration=CalibrationCapture(WEB/"hardware/runtime/astra-calibration",CAPTURE_SESSION)
    key=spec['id'];sequence=0;depth_files=collections.deque()
    for old in OUT.glob(f'{key}-z-*'):
        if time.time()-old.stat().st_mtime>60:old.unlink(missing_ok=True)
    while not stop.is_set():
        cap=None;depth_source=None;next_depth_retry=0;depth_error='Depth not started'
        try:
            publish(OUT,key,None,sequence,None,'connecting','正在连接相机')
            dev=select_device(spec,devices())
            if not dev:raise RuntimeError('未找到唯一匹配的相机')
            if subprocess.run(['fuser',dev],capture_output=True).returncode==0:
                raise RuntimeError('相机被其他程序占用')
            cap=cv2.VideoCapture(dev,cv2.CAP_V4L2)
            if not cap.isOpened():raise RuntimeError('相机无法打开')
            cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*spec['fourcc']))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
            cap.set(cv2.CAP_PROP_FPS,15)
            warmup=10;last=0
            while not stop.is_set():
                if calibration.poll(cap,depth_source is not None or control.enabled(key)):continue
                ok,frame=cap.read()
                captured_at=time.time()
                if not ok or frame is None:raise RuntimeError('未收到相机画面')
                if warmup:
                    warmup-=1;continue
                if time.monotonic()-last<.2:continue
                ok,jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,82])
                if not ok:raise RuntimeError('图像编码失败')
                sequence+=1
                requested=control.enabled(key)
                if not requested and depth_source is not None:
                    depth_source.close();depth_source=None
                depth_meta={'requested':requested,'state':'starting' if requested else 'disabled','message':depth_error if requested else ''}
                if requested and depth_source is None and time.monotonic()>=next_depth_retry:
                    try:depth_source=AstraDepth()
                    except Exception as error:
                        depth_error=error_message(error);depth_meta.update(state='error',message=depth_error);control.request({'camera':key,'enabled':False});next_depth_retry=time.monotonic()+3
                if depth_source is not None:
                    try:
                        raw,info=depth_source.read()
                        depth_meta,paths=write_depth(OUT,key,f'{SESSION}-{sequence}',raw,.001,depth_source.intrinsics,info['aligned_to_color'],info['frame_number'],info['sensor_timestamp_ms'])
                        depth_meta.update(info,requested=True);depth_files.append(paths)
                        while len(depth_files)>35:
                            for expired in depth_files.popleft():expired.unlink(missing_ok=True)
                    except Exception as error:
                        depth_error=error_message(error);depth_meta={'requested':False,'state':'error','message':depth_error};control.request({'camera':key,'enabled':False})
                        try:depth_source.close()
                        except Exception:pass
                        depth_source=None;next_depth_retry=time.monotonic()+3
                publish(OUT,key,jpeg.tobytes(),sequence,captured_at,'online',extra={'depth':depth_meta,'active_streams':['color','depth'] if depth_meta['state']=='online' else ['color']})
                last=time.monotonic()
        except Exception as error:
            publish(OUT,key,None,sequence,None,'offline',str(error),extra={'depth':{'requested':control.enabled(key),'state':'error' if control.enabled(key) else 'disabled','message':str(error)}})
        finally:
            if cap is not None:cap.release()
            if depth_source is not None:
                try:depth_source.close()
                except Exception:pass
        stop.wait(3)
    publish(OUT,key,None,sequence,None,'offline','视频采集已停止')

def parse_options(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera',choices=[spec['id'] for spec in SPECS],help='Capture only this camera')
    parser.add_argument('--skip-camera',choices=[spec['id'] for spec in SPECS],action='append',default=[],help='Leave a faulty camera unopened')
    parser.add_argument('--no-detector',action='store_true',default=True,help='Compatibility flag; recognition is disabled')
    return parser.parse_args(argv)

def main(argv=None):
    global CAPTURE_SESSION
    options=parse_options(argv)
    specs=[spec for spec in SPECS if (options.camera is None or spec['id']==options.camera) and spec['id'] not in options.skip_camera]
    logs=WEB/'logs';logs.mkdir(exist_ok=True)
    with (logs/'camera-preview.lock').open('a+') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('Camera preview already running')
        lock.seek(0);lock.truncate();lock.write(str(os.getpid()));lock.flush()
        control=DepthControl(WEB/'hardware/runtime/camera-depth-control.json',OUT)
        CAPTURE_SESSION=control.reset()
        for key in ['d435','d405','astra']:
            OUT.mkdir(parents=True,exist_ok=True)
            atomic(OUT/(key+'-battery.json'),json.dumps(dict(state='disabled',sequence=0,captured_at=None,message='YOLOE disabled',source_camera=key)).encode())
        stop=threading.Event()
        for sig in [signal.SIGTERM,signal.SIGINT]:signal.signal(sig,lambda *_:stop.set())
        for spec in SPECS:
            if spec not in specs:publish(OUT,spec['id'],None,0,None,'offline','本次未启用')
        workers=[threading.Thread(target=worker,args=(spec,stop,control),daemon=True) for spec in specs]
        for t in workers:t.start()
        print('Camera preview started:',','.join(spec['id'] for spec in specs),'color only; depth defaults OFF; YOLOE disabled',flush=True)
        try:
            while not stop.is_set():stop.wait(1)
        finally:
            stop.set()
            for t in workers:t.join(timeout=5)

if __name__=='__main__':main()
