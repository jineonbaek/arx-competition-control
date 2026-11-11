"""Reuse the existing 9983 D405 capture owner; archive one immutable scene."""
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

def save_frame(store,blob,*,source,expected_id=None,calibration_sha256=None):
    with np.load(io.BytesIO(blob),allow_pickle=False) as f:
        validate_frame(f)
        ident=str(f["id"])
        if expected_id is not None and ident!=expected_id:
            raise AgentError("stale_snapshot","快照已被更新，请重新采集")
        output=io.BytesIO();Image.fromarray(f["rgb"]).save(output,format="JPEG",quality=95)
        k=json.loads(str(f["intrinsics"]))
        manifest=dict(source=source,source_snapshot=ident,target_frame="base_link",
            scene_sha256=hashlib.sha256(blob).hexdigest(),calibration_sha256=calibration_sha256,
            arm_timestamp=float(f["arm_time"]),body_timestamp=float(f["body_time"]),
            cameras={"d405":dict(image_size_px=[k["width"],k["height"]],captured_at=float(f["image_time"]),
                intrinsics=k,geometry_ready=True)})
    return store.save_observation(manifest,{"d405":output.getvalue()},geometry_archive=blob)

def capture(registry):
    url=registry.config.console_url
    digest=hashlib.sha256(CALIBRATION.read_bytes()).hexdigest()
    req=Request(url+"/api/handle-preview",data=b'{"op":"capture"}',
        headers={"Origin":url,"Cookie":"arx_web_session="+str(uuid.uuid4()),"Content-Type":"application/json"})
    with urlopen(req,timeout=25) as response:view=json.load(response)
    blob=(ROOT/"web/hardware/runtime/grasp-preview/frame.npz").read_bytes()
    if hashlib.sha256(CALIBRATION.read_bytes()).hexdigest()!=digest:
        raise AgentError("calibration_changed","采集期间外参变化")
    return save_frame(registry.store,blob,source="9983_d405_paired",expected_id=view["id"],calibration_sha256=digest)

def frame_path(store,ident):
    obs=store.observation(ident)
    path=store.image_path(ident,"d405").parent/"scene.npz"
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=obs.get("scene_sha256"):
        raise AgentError("scene_changed","冻结深度不存在或已被改动")
    return path
