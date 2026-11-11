"""D405 file-only FoundationPose jobs. GPU work lives in a short-lived child."""
import base64
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import uuid

import cv2
import numpy as np
from grasp_preview import GraspPreview,roi_pixels

ROOT=Path(__file__).resolve().parents[2]
DEMO=Path.home()/"arx-foundationpose/foundationpose"
CACHE=ROOT/"web/hardware/runtime/foundationpose"


class FoundationPosePreview:
    def __init__(self,controller,depth,cache=CACHE):
        self.cache=Path(cache)
        self.capture=GraspPreview(controller,depth,self.cache/"capture")
        self.lock=threading.RLock()
        self.process=None
        self.owner=None
        self.job=None
        self.started=0.
        self.closed=False

    def _stop(self):
        if self.process is not None:
            process=self.process
            if process.poll() is None:
                try:os.killpg(process.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait(timeout=3)
            self.process=None

    def _refresh(self):
        if self.process is None:return
        code=self.process.poll()
        if code is None and time.monotonic()-self.started>90:
            self._stop()
            self.job.update(status="error",message="推理超时，进程已停止并释放显存")
        elif code is not None:
            self.process=None
            try:
                if code!=0:raise ValueError("推理未完成，请调整完整电池选框后重试")
                result=json.loads((self.cache/self.job["run_id"]/"result.json").read_text())
                if result["id"]!=self.job["id"]:raise ValueError("结果与快照不一致")
                transform=np.asarray(result["T_base_from_object"],float)
                if transform.shape!=(4,4) or not np.isfinite(transform).all():raise ValueError("位姿结果无效")
                self.job.update(status="complete",message="估计完成，推理进程已退出，显存已释放",result=result)
            except (OSError,KeyError,ValueError):
                self.job.update(status="error",message="估计失败，显存已释放；请检查完整电池选框、轮廓及有效深度")

    def snapshot(self):
        with self.lock:
            self._refresh()
            return dict(self.job or dict(status="idle",message="点击估计位姿时自动加载模型"),
                        running=self.process is not None)

    def _watch(self):
        while True:
            time.sleep(.5)
            with self.lock:
                self._refresh()
                if self.process is None:return

    def action(self,client,data):
        if not isinstance(data,dict):raise ValueError("无效请求")
        op=data.get("op")
        fields={"capture":{"op"},"infer":{"op","id","roi"},"stop":{"op","run_id"}}
        if op not in fields or set(data)!=fields[op]:raise ValueError("位姿请求字段无效")
        with self.lock:
            self._refresh()
            if self.closed:raise ValueError("网页服务正在关闭")
            if op=="stop":
                if not self.job or self.owner!=client or data["run_id"]!=self.job["run_id"]:
                    raise ValueError("位姿任务已更新，请刷新页面")
                self._stop()
                self.job.update(status="stopped",message="已取消估计，显存已释放")
                return self.snapshot()
            if self.process is not None:raise ValueError("已有位姿估计正在运行")
            if op=="capture":
                view=self.capture.action({"op":"capture"})
                with np.load(self.capture.cache/"frame.npz",allow_pickle=False) as frame:
                    depth=frame["raw_depth"].astype(float)*float(frame["scale"])
                valid=(depth>.07)&(depth<1.5)
                colors=cv2.applyColorMap(np.clip((depth-.07)/1.43*255,0,255).astype(np.uint8),cv2.COLORMAP_TURBO)
                colors[~valid]=0
                image=cv2.imencode(".jpg",colors)[1]
                self.job=dict(status="captured",message="已冻结 RGB-D，请框住完整电池",
                    view={key:view[key] for key in ("id","time","width","height","image","paired_ms")},
                    depth_image="data:image/jpeg;base64,"+base64.b64encode(image).decode(),
                    valid_depth_fraction=float(valid.mean()))
                self.owner=client
                return self.snapshot()
            if not self.job or self.owner!=client or not self.job.get("view") or data["id"]!=self.job["view"]["id"]:
                raise ValueError("快照已更新，请重新采集")
            view=self.job["view"]
            roi=roi_pixels(data["roi"],view["width"],view["height"])
            runner=DEMO/"run-snapshot.sh"
            if not runner.is_file():raise ValueError("FoundationPose 环境未安装")
            run_id=uuid.uuid4().hex
            output=self.cache/run_id
            output.mkdir(parents=True)
            shutil.copyfile(self.capture.cache/"frame.npz",output/"frame.npz")
            command=["bash",str(runner),"--frame",str(output/"frame.npz"),"--id",view["id"],
                     "--roi",*map(str,roi),"--output",str(output/"result.json")]
            with (output/"worker.log").open("wb") as log:
                self.process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
            self.started=time.monotonic()
            self.job.update(status="running",message="正在加载模型并估计位姿，完成后自动释放显存",
                            run_id=run_id,id=view["id"],roi=roi,result=None)
            threading.Thread(target=self._watch,daemon=True).start()
            return self.snapshot()

    def close(self):
        with self.lock:
            self.closed=True
            self._stop()
