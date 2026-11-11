"""Serial perception workers over a frozen observation, with bounded cancellation."""
import base64
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
from PIL import Image
from .contracts import AgentError
from .storage import atomic_json
from .grasp_scene import frame_path
from .grasp_geometry import ROOT,PROFILE,preview_geometry

VISION=Path("/home/arx/arx-vision-demo/vision/demo")
FP=Path("/home/arx/arx-foundationpose/foundationpose")

def run_worker(command,cwd,log,cancelled,timeout):
    if cancelled():raise AgentError("cancelled","预览已取消")
    started=time.monotonic()
    with log.open("w") as output:
        child=subprocess.Popen(command,cwd=cwd,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while child.poll() is None:
                if cancelled():raise AgentError("cancelled","预览已取消")
                if time.monotonic()-started>timeout:raise AgentError("worker_timeout","模型处理超时")
                time.sleep(.1)
            if cancelled():raise AgentError("cancelled","预览已取消")
            return child.returncode
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait()

@contextmanager
def gpu_slot():
    # Shared by integrated recognition and pose; standalone diagnostic pages use their own workers.
    with Path("/tmp/arx-grasp-integration-gpu.lock").open("a") as handle:
        try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise AgentError("gpu_busy","另一项整合任务正在使用模型")
        yield


def segment(store,ident,output,cancelled):
    camera=store.observation(ident).get("camera_id","d405")
    image=store.image_path(ident,camera)
    log=output/"recognition.log"
    code=run_worker([str(VISION/".venv/bin/python"),str(VISION/"pipeline.py"),
        "--image",str(image),"--attempts","1"],VISION,log,cancelled,420)
    if code not in (0,2):raise AgentError("vision_failed","电池识别失败；请检查本次 recognition.log")
    # Existing recognition CLI emits the JSON receipt as its final line.
    report=json.loads(log.read_text().strip().splitlines()[-1])
    source=Path(report["output"]).resolve()
    if not source.is_relative_to((VISION/"runtime").resolve()):raise ValueError("Unknown recognition artifact")
    result={key:report.get(key) for key in ("status","selection","coarse_selection","review","path","elapsed_s","peak_gpu_mib")}
    if report["status"]=="perception_accepted":
        mask=np.asarray(Image.open(source/"mask.png").convert("L"))
        with Image.open(image) as original:
            if mask.shape!=(original.height,original.width):raise ValueError("Mask size mismatch")
        y,x=np.nonzero(mask)
        if len(x)<100:raise ValueError("Empty battery mask")
        result["bbox_xyxy"]=[int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)]
        (output/"mask.png").write_bytes((source/"mask.png").read_bytes())
    for name in ("overlay.png","candidates.png","rgb.png"):
        if (source/name).is_file():
            result["overlay"]="data:image/png;base64,"+base64.b64encode((source/name).read_bytes()).decode()
            break
    return result

def estimate(store,ident,recognition,output,cancelled):
    if store.observation(ident).get("camera_id","d405")!="d405":
        raise ValueError("Fine grasp estimation requires a D405 observation")
    frame=frame_path(store,ident)
    with np.load(frame,allow_pickle=False) as f:source_id=str(f["id"])
    result=output/"pose.json"
    command=["bash","-c",'source "$1"; exec "$2" "$3" "${@:4}"',"pose-worker",str(FP/"env.sh"),
        str(FP/".venv/bin/python"),str(ROOT/"foundationpose/infer_snapshot.py"),
        "--frame",str(frame),"--id",source_id,"--roi",*map(str,recognition["bbox_xyxy"]),
        "--mask",str(output/"mask.png"),"--output",str(result)]
    code=run_worker(command,FP,output/"pose.log",cancelled,120)
    if code:raise AgentError("pose_failed","FoundationPose 失败；请检查本次 pose.log")
    report=json.loads(result.read_text())
    if report.get("id")!=source_id or report.get("roi")!=recognition["bbox_xyxy"]:
        raise ValueError("Pose receipt does not match recognition")
    return report

def pipeline(store,ident,clearance_mm,output,progress=lambda stage:None):
    output.mkdir(parents=True,exist_ok=True)
    cancelled=lambda:store.cancelled
    frame=frame_path(store,ident)
    with gpu_slot():
        progress("recognition")
        relative=store.state.get('battery_observations',{}).get(ident)
        if relative:
            observed=(store.path/relative).resolve()
            if not observed.is_relative_to(store.path.resolve()):raise ValueError("Invalid observation receipt path")
            receipt=json.loads(observed.read_text())
            if receipt.get('observation_id')!=ident or receipt.get('camera')!='d405':
                raise ValueError("Recognition receipt identity mismatch")
            recognition=receipt['recognition']
            if recognition.get('status')=='perception_accepted':
                (output/'mask.png').write_bytes((observed.parent/'mask.png').read_bytes())
        else:recognition=segment(store,ident,output,cancelled)
        result=dict(observation_id=ident,recognition=recognition,executable=False,tcp_status="pending")
        if recognition["status"]!="perception_accepted":
            result.update(status="recognition_rejected",message="识别未通过复核，本次没有估计位姿或生成路径。")
        else:
            progress("foundationpose")
            pose=estimate(store,ident,recognition,output,cancelled)
            mesh=ROOT/"web/cad/gripper-v2/battery-print.stl"
            if pose.get("mesh_sha256")!=hashlib.sha256(mesh.read_bytes()).hexdigest():
                raise ValueError("Pose CAD mismatch")
            profile=json.loads(PROFILE.read_text())
            if profile["mesh_sha256"]["battery-print.stl"]!=pose["mesh_sha256"]:raise ValueError("Grip profile CAD mismatch")
            progress("pointcloud_path")
            with np.load(frame,allow_pickle=False) as f:
                geometry=preview_geometry(f,np.asarray(Image.open(output/"mask.png").convert("L")),pose,clearance_mm)
            result.update(status="preview_complete",pose=pose,geometry=geometry,
                message="识别、位姿、点云与接触点路径已串联；等待 TCP 与运动验证。")
        if cancelled():raise AgentError("cancelled","预览已取消")
        atomic_json(output/"result.json",result)
        progress(result["status"])
        return result
