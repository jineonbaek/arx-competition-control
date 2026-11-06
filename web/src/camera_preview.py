#!/usr/bin/env python3
"""Camera-only latest JPEG publisher. No ROS, robot SDK or CAN access."""
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

WEB=Path(__file__).resolve().parents[1]
OUT=WEB/'public'/'camera-live'
SESSION=uuid.uuid4().hex[:10]
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
    metadata=dict(state=state,sequence=sequence,captured_at=captured_at,message=message)
    if jpeg is not None and key in RECENT:
        name=f'{key}-v-{SESSION}-{sequence}.jpg';p=directory/name
        atomic(p,jpeg);metadata['image_file']=name;RECENT[key].append(p)
        while len(RECENT[key])>35:RECENT[key].popleft().unlink(missing_ok=True)
    if extra:metadata.update(extra)
    atomic(directory/(key+'.json'),json.dumps(metadata,allow_nan=False).encode())

def worker(spec,stop):
    if spec['id']=='d435':
        from battery_rgbd import run
        return run(spec,stop,publish,OUT)
    import cv2
    key=spec['id'];sequence=0
    while not stop.is_set():
        cap=None
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
                ok,frame=cap.read()
                captured_at=time.time()
                if not ok or frame is None:raise RuntimeError('未收到相机画面')
                if warmup:
                    warmup-=1;continue
                if time.monotonic()-last<.2:continue
                ok,jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,82])
                if not ok:raise RuntimeError('图像编码失败')
                sequence+=1
                publish(OUT,key,jpeg.tobytes(),sequence,captured_at,'online')
                last=time.monotonic()
        except Exception as error:
            publish(OUT,key,None,sequence,None,'offline',str(error))
        finally:
            if cap is not None:cap.release()
        stop.wait(3)
    publish(OUT,key,None,sequence,None,'offline','视频采集已停止')

def main():
    logs=WEB/'logs';logs.mkdir(exist_ok=True)
    with (logs/'camera-preview.lock').open('a+') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('Camera preview already running')
        lock.seek(0);lock.truncate();lock.write(str(os.getpid()));lock.flush()
        stop=threading.Event()
        for sig in [signal.SIGTERM,signal.SIGINT]:signal.signal(sig,lambda *_:stop.set())
        workers=[threading.Thread(target=worker,args=(spec,stop),daemon=True) for spec in SPECS]
        for t in workers:t.start()
        print('Camera preview started: 3 RGB cameras, 640x480, up to 5 JPEG frames/s; no robot interfaces',flush=True)
        detector=None;next_retry=0.0
        vision=WEB.parent/'vision'
        command=[str(vision/'.venv-detector/bin/python'),str(WEB/'src/battery_boxes_service.py')]
        try:
            while not stop.is_set():
                if (detector is None or detector.poll() is not None) and time.monotonic()>=next_retry:
                    if all(Path(path).is_file() for path in command) and (vision/'models/yoloe-11s-seg.pt').is_file():
                        detector=subprocess.Popen(command,cwd=WEB.parent,stdin=subprocess.DEVNULL,start_new_session=True)
                        (vision/'runtime').mkdir(exist_ok=True)
                        (vision/'runtime/battery-boxes.pid').write_text(str(detector.pid))
                        print('Whole battery child started',detector.pid,flush=True)
                    next_retry=time.monotonic()+10
                stop.wait(1)
        finally:
            stop.set()
            if detector is not None and detector.poll() is None:
                detector.terminate()
                try:detector.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    detector.kill();detector.wait(timeout=2)
            for t in workers:t.join(timeout=2)
            for spec in SPECS:publish(OUT,spec['id'],None,0,None,'offline','视频采集已停止')

if __name__=='__main__':main()
