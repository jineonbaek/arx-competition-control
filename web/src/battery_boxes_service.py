#!/usr/bin/env python3
"""Read-only per-camera reference detector. Reads camera JPEGs; writes only overlays."""
from pathlib import Path
import collections
import fcntl
import json
import os
import signal
import threading
import time
import urllib.request
import uuid

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'web/public/camera-live'
RUNTIME=ROOT/'vision/runtime'
os.environ['YOLO_CONFIG_DIR']=str(RUNTIME/'ultralytics')
os.environ['YOLO_AUTOINSTALL']='false'
import cv2
import numpy as np
import torch
from battery_boxes import filter_boxes
from battery_features import associate_holes,draw_features,bind_measurement


def atomic(path,data):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_bytes(data);os.replace(tmp,path)

def state(key,sequence,state='offline',captured=None,message='',**extra):
    data=dict(state=state,sequence=sequence,captured_at=captured,message=message,source_camera=key)
    data.update(extra);atomic(OUT/(key+'-battery.json'),json.dumps(data,allow_nan=False).encode())

def load_frame(key):
    with urllib.request.urlopen(f'http://127.0.0.1:9983/camera-live/{key}.json',timeout=2) as response:meta=json.load(response)
    if meta['state']!='online' or not 0<=time.time()-meta['captured_at']<1.5:raise RuntimeError('相机画面无效或过期')
    name=meta.get('raw_image_file',meta.get('image_file',key+'.jpg'))
    if Path(name).name!=name or not name.startswith(key):raise RuntimeError('Invalid camera image path')
    with urllib.request.urlopen('http://127.0.0.1:9983/camera-live/'+name,timeout=2) as response:body=response.read()
    image=cv2.imdecode(np.frombuffer(body,np.uint8),cv2.IMREAD_COLOR)
    if image is None:raise RuntimeError('相机图像无法解码')
    return meta,image

def main():
    from ultralytics import YOLOE,settings
    settings.update({'sync':False})
    torch.set_num_threads(4);torch.set_num_interop_threads(1)
    RUNTIME.mkdir(exist_ok=True,parents=True);OUT.mkdir(exist_ok=True,parents=True)
    with (RUNTIME/'battery-boxes.lock').open('a+') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('Whole battery detector already running')
        lock.seek(0);lock.truncate();lock.write(str(os.getpid()));lock.flush()
        stop=threading.Event()
        for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,lambda *_:stop.set())
        config=json.loads((ROOT/'vision/config/whole-battery-references.json').read_text())
        models={};counters={key:0 for key in config['references']};last={};files={key:collections.deque() for key in counters};session=uuid.uuid4().hex[:10]
        for key,spec in config['references'].items():
            state(key,0,'connecting',message='整电池识别初始化')
            model=YOLOE(str(ROOT/'vision/models/yoloe-11s-seg.pt'))
            ref=str(ROOT/'vision/config'/spec['image']);prompts={'bboxes':np.array(spec['boxes']),'cls':np.array(spec['classes'])}
            model.predict(ref,refer_image=ref,visual_prompts=prompts,imgsz=640,conf=.05,iou=.5,device='cpu',verbose=False)
            models[key]=model
            for old in OUT.glob(key+'-b-*.jpg'):
                if time.time()-old.stat().st_mtime>60:old.unlink(missing_ok=True)
        print('Battery body + visible-hole detector ready: local CPU; all 3 cameras; no robot interfaces',flush=True)
        try:
            while not stop.is_set():
                round_start=time.monotonic()
                for key,model in models.items():
                    if stop.is_set():break
                    try:
                        meta,image=load_frame(key);frame_key=(meta.get('image_file'),meta['sequence'],meta['captured_at'])
                        if last.get(key)==frame_key:continue
                        start=time.monotonic()
                        result=model.predict(image,imgsz=640,conf=.10,iou=.5,device='cpu',verbose=False)[0]
                        rows=[dict(xyxy=b.xyxy[0].tolist(),score=float(b.conf[0]),class_id=int(b.cls[0])) for b in result.boxes]
                        boxes=filter_boxes(rows,image.shape[1],image.shape[0],config['minimum_score'])
                        elapsed=(time.monotonic()-start)*1000
                        if time.time()-meta['captured_at']>2.4:raise RuntimeError('识别结果过期，显示原始预览')
                        holes=associate_holes(image,boxes)
                        measurement=bind_measurement(meta.get('battery'),holes)
                        annotated=draw_features(image,boxes,holes,measurement);ok,jpeg=cv2.imencode('.jpg',annotated,[cv2.IMWRITE_JPEG_QUALITY,86])
                        if not ok:raise RuntimeError('框选图像编码失败')
                        counters[key]+=1;name=f'{key}-b-{session}-{counters[key]}.jpg';p=OUT/name;atomic(p,jpeg.tobytes());files[key].append(p)
                        features=dict(boxes=boxes,count=len(boxes),partial_count=sum(x['partial'] for x in boxes),holes=holes,hole_count=len(holes),hole_status='observed' if holes else 'not_visible_or_not_detected',method='YOLOE_visual_reference_and_visible_slot_geometry',minimum_score=config['minimum_score'],inference_ms=elapsed,source_sequence=meta['sequence'],coordinate_kind='image_pixels_only',read_only=True)
                        extras={'battery':measurement} if key=='d435' and measurement else {}
                        state(key,counters[key],'online',meta['captured_at'],image_file=name,whole_battery=features,**extras)
                        while len(files[key])>35:files[key].popleft().unlink(missing_ok=True)
                        last[key]=frame_key
                    except Exception as error:
                        state(key,counters[key],'offline',message=str(error));print(key,'detector retry',repr(error),flush=True)
                stop.wait(max(0,.5-(time.monotonic()-round_start)))
        finally:
            for key in models:state(key,counters[key],'offline',message='整电池识别已停止')

if __name__=='__main__':main()
