"""Archive paired RGB-D through the existing 9983 camera owner."""
import hashlib
import io
import json
from pathlib import Path
from urllib.request import Request,urlopen
import uuid
import numpy as np
from PIL import Image
from .contracts import AgentError
from .grasp_geometry import validate_frame

ROOT=Path(__file__).resolve().parents[1]
CALIBRATION=ROOT/"vision/handeye/results/d405-right-cc200-ee0ff2fa49e9e56285d271d6.json"

def save_frame(store,blob,*,source,expected_id=None,calibration_sha256=None,camera="d405"):
    if camera not in ("d435","d405"):raise ValueError("Unsupported camera")
    with np.load(io.BytesIO(blob),allow_pickle=False) as f:
        validate_frame(f)
        actual=str(f["camera_id"]) if "camera_id" in f else "d405"
        if actual!=camera:raise ValueError("RGB-D camera identity mismatch")
        if "calibration_sha256" in f and calibration_sha256 is not None and str(f["calibration_sha256"])!=calibration_sha256:
            raise ValueError("RGB-D calibration identity mismatch")
        ident=str(f["id"])
        if expected_id is not None and ident!=expected_id:
            raise AgentError("stale_snapshot","快照已被更新，请重新采集")
        output=io.BytesIO();Image.fromarray(f["rgb"]).save(output,format="JPEG",quality=95)
        k=json.loads(str(f["intrinsics"]))
        manifest=dict(source=source,camera_id=camera,source_snapshot=ident,target_frame="base_link",
            scene_sha256=hashlib.sha256(blob).hexdigest(),calibration_sha256=calibration_sha256,
            arm_timestamp=float(f["arm_time"]),body_timestamp=float(f["body_time"]),
            cameras={camera:dict(image_size_px=[k["width"],k["height"]],captured_at=float(f["image_time"]),
                intrinsics=k,geometry_ready=True)})
    return store.save_observation(manifest,{camera:output.getvalue()},geometry_archive=blob)

def capture(registry,camera="d405"):
    if camera not in ("d435","d405"):raise ValueError("Unsupported camera")
    calibration=CALIBRATION if camera=="d405" else ROOT/"vision/handeye/results/d435-cad-349622074226.json"
    url=registry.config.console_url
    digest=hashlib.sha256(calibration.read_bytes()).hexdigest()
    endpoint="/api/handle-preview" if camera=="d405" else "/api/grasp-scene"
    data={"op":"capture"} if camera=="d405" else {"camera":camera}
    req=Request(url+endpoint,data=json.dumps(data).encode(),
        headers={"Origin":url,"Cookie":"arx_web_session="+str(uuid.uuid4()),"Content-Type":"application/json"})
    with urlopen(req,timeout=25) as response:view=json.load(response)
    cache="grasp-preview" if camera=="d405" else "grasp-head"
    blob=(ROOT/"web/hardware/runtime"/cache/"frame.npz").read_bytes()
    if hashlib.sha256(calibration.read_bytes()).hexdigest()!=digest:
        raise AgentError("calibration_changed","采集期间外参变化")
    return save_frame(registry.store,blob,source="9983_"+camera+"_paired",expected_id=view["id"],calibration_sha256=digest,camera=camera)

def frame_path(store,ident):
    obs=store.observation(ident)
    camera=obs.get("camera_id","d405")
    path=store.image_path(ident,camera).parent/"scene.npz"
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=obs.get("scene_sha256"):
        raise AgentError("scene_changed","冻结深度不存在或已被改动")
    return path
