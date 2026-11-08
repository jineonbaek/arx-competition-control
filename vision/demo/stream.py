"""Continuous local overlays plus asynchronous Qwen snapshot review; start on demand."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import threading
import time
from io import BytesIO
from urllib.request import urlopen

import numpy as np
from PIL import Image
from models import VisionModels
from pipeline import QwenJudge, run_frame, overlay_candidates
from target import DINO_PROMPT, TASK, load_reference, shape_view

def atomic_json(path, data):
    temporary=path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False))
    os.replace(temporary,path)

def read_frame(camera, last):
    base="http://127.0.0.1:9983/camera-live/"
    with urlopen(base+camera+".json",timeout=3) as response:
        meta=json.load(response)
    identity=(meta.get("capture_session"),meta.get("sequence"))
    if meta.get("state")!="online" or not -.2<=time.time()-meta.get("captured_at",0)<=2:
        raise ValueError("相机画面离线或过期")
    if identity==last:
        return None
    name=meta.get("raw_image_file") or meta.get("image_file")
    if not isinstance(name,str) or Path(name).name!=name or not name.startswith(camera+"-"):
        raise ValueError("相机图像引用无效")
    with urlopen(base+name,timeout=3) as response:
        data=response.read(8_000_001)
    if len(data)>8_000_000:
        raise ValueError("相机图像过大")
    return identity,meta,Image.open(BytesIO(data)).convert("RGB")

class LockedModels:
    def __init__(self,models,lock):
        self.models,self.lock=models,lock
    def detect(self,*args,**kwargs):
        with self.lock:
            return self.models.detect(*args,**kwargs)
    def segment(self,*args,**kwargs):
        with self.lock:
            return self.models.segment(*args,**kwargs)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--camera",required=True,choices=["d435","d405"])
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    stop=threading.Event()
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    state={"phase":"loading","sequence":0,"camera":args.camera,"message":"正在加载模型"}
    atomic_json(output/"state.json",state)
    models=VisionModels()
    model_lock=threading.Lock()
    locked=LockedModels(models,model_lock)
    reference=load_reference()
    state_lock=threading.Lock()
    cloud={"phase":"waiting","message":"等待首帧"}
    cloud_thread=None
    last_cloud=0
    last=None
    recent=deque()
    last_published=None

    def review(image,meta,sequence):
        nonlocal cloud
        directory=output/("review-"+str(sequence))
        directory.mkdir()
        image.save(directory/"rgb.png")
        try:
            judge=QwenJudge(Path("/home/arx/arx-competition-control/agent/.secrets/qwen.env"))
            result=run_frame(locked,judge,image,reference,TASK,directory)
            overlay=directory/"overlay.png"
            if overlay.exists():
                Image.open(overlay).convert("RGB").save(directory/"review.jpg",quality=88)
            else:
                image.save(directory/"review.jpg",quality=88)
            summary={"phase":"done","source_sequence":sequence,"captured_at":meta["captured_at"],
                     "image_file":str(directory.relative_to(output)/"review.jpg"),
                     "status":result["status"],"path":result.get("path"),
                     "selection":result.get("selection") or result.get("coarse_selection"),
                     "review":result.get("review")}
            atomic_json(directory/"result.json",result)
        except Exception:
            summary={"phase":"error","source_sequence":sequence,"captured_at":meta["captured_at"],
                     "message":"Qwen复核暂不可用，稍后重新尝试"}
        with state_lock:
            cloud=summary

    try:
        while not stop.is_set():
            lease=json.loads((output/"lease.json").read_text())
            if time.time()>lease["expires_at"]:
                break
            started=time.monotonic()
            try:
                sample=read_frame(args.camera,last)
                if sample is None:
                    stop.wait(.1)
                    continue
                last,meta,image=sample
                with model_lock:
                    shape=shape_view(image)
                    candidates=models.detect(shape,DINO_PROMPT)
                    pixels=np.asarray(image).copy()
                    for row in candidates[:4]:
                        mask,_=models.segment(shape,box=row["bbox_xyxy"])
                        pixels[mask]=(pixels[mask]*.6+np.array([0,95,65])).clip(0,255).astype(np.uint8)
                frame=overlay_candidates(Image.fromarray(pixels),candidates)
                sequence=state["sequence"]+1
                name="frame-"+str(sequence)+".jpg"
                frame.save(output/name,quality=85)
                recent.append(output/name)
                while len(recent)>6:
                    recent.popleft().unlink(missing_ok=True)
                if cloud_thread is None or not cloud_thread.is_alive():
                    if time.monotonic()-last_cloud>=12:
                        last_cloud=time.monotonic()
                        with state_lock:
                            cloud={**cloud,"phase":"thinking","message":"正在复核抽样画面"}
                        cloud_thread=threading.Thread(target=review,args=(image.copy(),dict(meta),sequence),daemon=True)
                        cloud_thread.start()
                with state_lock:
                    summary=dict(cloud)
                published=time.monotonic()
                interval=published-(last_published if last_published is not None else started)
                last_published=published
                state.update(phase="streaming",sequence=sequence,image_file=name,
                             captured_at=meta["captured_at"],fps=round(1/max(.001,interval),1),
                             count=len(candidates),qwen=summary,message="实时候选与分割")
                atomic_json(output/"state.json",state)
                # Keep only completed review artifacts for the latest two samples.
                if cloud_thread is not None and not cloud_thread.is_alive():
                    directories=sorted(output.glob("review-*"),key=lambda p:p.stat().st_mtime)
                    for old in directories[:-2]:
                        import shutil
                        assert old.parent==output and old.name.startswith("review-")
                        shutil.rmtree(old)
            except Exception:
                state.update(phase="waiting",message="等待新的相机画面")
                atomic_json(output/"state.json",state)
                stop.wait(.5)
            stop.wait(max(0,.4-(time.monotonic()-started)))
    finally:
        state.update(phase="stopped",message="识别已停止")
        atomic_json(output/"state.json",state)

if __name__=="__main__":
    main()
