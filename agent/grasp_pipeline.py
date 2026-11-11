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

def estimate(store,ident,recognition,output,cancelled,head=None):
    if store.observation(ident).get("camera_id","d405")!="d405":
        raise ValueError("Fine grasp estimation requires a D405 observation")
    frame=frame_path(store,ident)
    with np.load(frame,allow_pickle=False) as f:source_id=str(f["id"])
    result=output/"pose.json"
    command=["bash","-c",'source "$1"; exec "$2" "$3" "${@:4}"',"pose-worker",str(FP/"env.sh"),
        str(FP/".venv/bin/python"),str(ROOT/"foundationpose/infer_snapshot.py"),
        "--frame",str(frame),"--id",source_id,"--roi",*map(str,recognition["bbox_xyxy"]),
        "--mask",str(output/"mask.png"),"--output",str(result)]
    if head:
        command += ['--head-frame',head['frame'],'--head-mask',head['mask'],'--head-id',head['snapshot_id']]
    code=run_worker(command,FP,output/"pose.log",cancelled,120)
    if code:raise AgentError("pose_failed","FoundationPose 失败；请检查本次 pose.log")
    report=json.loads(result.read_text())
    if report.get("id")!=source_id or report.get("roi")!=recognition["bbox_xyxy"]:
        raise ValueError("Pose receipt does not match recognition")
    return report

def head_evidence(store,ident,output):
    observation=store.observation(ident)
    if observation.get('camera_id')!='d435':raise ValueError('Handoff requires D435 evidence')
    relative=store.state.get('battery_observations',{}).get(ident)
    if not relative:raise ValueError('Missing head recognition receipt')
    path=(store.path/relative).resolve()
    if not path.is_relative_to(store.path.resolve()):raise ValueError('Invalid head evidence path')
    receipt=json.loads(path.read_text())
    if (receipt.get('observation_id')!=ident or receipt.get('camera')!='d435'
            or receipt.get('status')!='localized'
            or receipt.get('recognition',{}).get('status')!='perception_accepted'):
        raise ValueError('Head target has not been localized')
    archive=frame_path(store,ident)
    mask_path=path.parent/'mask.png'
    if not mask_path.resolve().is_relative_to(store.path.resolve()):raise ValueError('Invalid head mask path')
    mask_blob=mask_path.read_bytes()
    frozen=output/'head-mask.png';frozen.write_bytes(mask_blob)
    return dict(observation_id=ident,frame=str(archive),mask=str(frozen),
        snapshot_id=observation['source_snapshot'],scene_sha256=observation['scene_sha256'],
        mask_sha256=hashlib.sha256(mask_blob).hexdigest())


def pipeline(store,ident,clearance_mm,output,progress=lambda stage:None,*,head_observation_id=None):
    output.mkdir(parents=True,exist_ok=True)
    cancelled=lambda:store.cancelled
    frame=frame_path(store,ident)
    head=head_evidence(store,head_observation_id,output) if head_observation_id else None
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
            pose=estimate(store,ident,recognition,output,cancelled,head=head)
            mesh=ROOT/"web/cad/gripper-v2/battery-print.stl"
            if pose.get("mesh_sha256")!=hashlib.sha256(mesh.read_bytes()).hexdigest():
                raise ValueError("Pose CAD mismatch")
            profile=json.loads(PROFILE.read_text())
            if profile["mesh_sha256"]["battery-print.stl"]!=pose["mesh_sha256"]:raise ValueError("Grip profile CAD mismatch")
            quality=pose.get('quality')
            result.update(pose=pose,quality=quality)
            if not isinstance(quality,dict) or quality.get('version')!=1 or quality.get('status')!='consistent':
                result.update(status='pose_rejected',message='CAD 位姿与深度表面核对未通过；保留识别和位姿叠加，不生成抓取线段。')
            elif head and (not isinstance(pose.get('camera_handoff'),dict)
                    or pose['camera_handoff'].get('head_snapshot_id')!=head['snapshot_id']
                    or pose['camera_handoff'].get('head_scene_sha256')!=head['scene_sha256']
                    or pose['camera_handoff'].get('head_mask_sha256')!=head['mask_sha256']):
                raise ValueError('Cross-camera receipt does not match frozen head evidence')
            elif head and (pose['camera_handoff'].get('quality',{}).get('version')!=1
                    or pose['camera_handoff'].get('quality',{}).get('status')!='consistent'):
                result.update(status='handoff_rejected',camera_handoff=pose['camera_handoff'],
                    message='D405 位姿投回 D435 后与原目标不一致或证据不足；未生成抓取线段。')
            else:
                progress("pointcloud_path")
                with np.load(frame,allow_pickle=False) as f:
                    geometry=preview_geometry(f,np.asarray(Image.open(output/"mask.png").convert("L")),pose,clearance_mm)
                result.update(status="preview_complete",geometry=geometry,
                    message="识别、位姿表面核对、点云与接触点路径已串联；等待 TCP 与运动验证。")
        if head and result.get('pose',{}).get('camera_handoff'):
            result['camera_handoff']={**pose['camera_handoff'],'head_observation_id':head['observation_id'],
                'wrist_observation_id':ident}
        if cancelled():raise AgentError("cancelled","预览已取消")
        atomic_json(output/"result.json",result)
        progress(result["status"])
        return result
