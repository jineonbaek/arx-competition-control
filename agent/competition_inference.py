import json
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image
from .competition_scene import GRASP, VISION, MAIN, scene
from .contracts import AgentError


def segment(store, ident, camera, output):
    image=store.image_path(ident,camera)
    command=[str(VISION/".venv/bin/python"),str(VISION/"pipeline.py"),"--image",str(image),"--attempts","1"]
    run=subprocess.run(command,cwd=VISION,capture_output=True,text=True,timeout=420)
    if run.returncode not in (0,2):
        raise AgentError("vision_failed","视觉模型或 Qwen 复核失败，请检查识别环境")
    report=json.loads(run.stdout.strip().splitlines()[-1])
    source=Path(report["output"]).resolve()
    if not source.is_relative_to((VISION/"runtime").resolve()):
        raise AgentError("invalid_artifact","未知分割结果目录")
    result={"observation_id":ident,"camera":camera,"status":report["status"],
            "selection":report.get("selection"),"coarse_selection":report.get("coarse_selection"),
            "review":report.get("review"),"path":report.get("path")}
    if report["status"]=="perception_accepted":
        mask=np.asarray(Image.open(source/"mask.png").convert("L"))
        with Image.open(image) as original:
            if mask.shape!= (original.height,original.width):
                raise AgentError("mask_size","分割尺寸不匹配")
        ys,xs=np.nonzero(mask)
        if not len(xs):
            raise AgentError("empty_mask","分割区域为空")
        result["bbox_xyxy"]=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)]
        (output/"mask.png").write_bytes((source/"mask.png").read_bytes())
    for name in ("overlay.png","candidates.png"):
        if (source/name).is_file():
            (output/name).write_bytes((source/name).read_bytes())
    return result


def grasp_candidates(store, ident, receipt, output):
    info=receipt.get("result") or {}
    if (receipt["tool"]!="locate.segment" or info.get("observation_id")!=ident or
        info.get("camera")!="d405" or info.get("status")!="perception_accepted"):
        raise AgentError("unverified_mask","需要同一 D405 快照通过复核的分割任务")
    frame=scene(store,ident)/"scene.npz"
    with np.load(frame,allow_pickle=False) as data:
        frame_id=str(data["id"])
    result=output/"grasp.json"
    box=info["bbox_xyxy"]
    worker=Path(__file__).resolve().parents[1]/"vision/grasp/infer_d405.py"
    run=subprocess.run([str(GRASP/".venv/bin/python"),str(worker),str(GRASP),str(frame),frame_id,
                        str(result),*map(str,box),str(store.path/"jobs"/receipt["job_id"]/"mask.png")],capture_output=True,text=True,timeout=120)
    if run.returncode:
        raise AgentError("grasp_failed","GraspNet 未产生可用输出，请检查有效深度和模型环境")
    report=json.loads(result.read_text())
    if report["id"]!=frame_id or report["roi"]!=box:
        raise AgentError("snapshot_mismatch","抓取结果不属于当前快照")
    from PIL import ImageDraw
    preview=Image.open(store.image_path(ident,"d405")).convert("RGB")
    draw=ImageDraw.Draw(preview)
    for index,candidate in enumerate(report["candidates"]):
        pts=candidate["pixels"]
        for i,j in ((0,1),(1,2),(2,3),(4,5)):
            if pts[i] and pts[j]:
                draw.line([tuple(pts[i]),tuple(pts[j])],fill="yellow" if index==0 else "cyan",width=2)
    preview.save(output/"overlay.png")
    return {"observation_id":ident,"segment_job":receipt["job_id"],"frame":"body4",
            "candidates":[{k:c[k] for k in ("score","width_mm","body4_mm","rotation_body4")} for c in report["candidates"]],
            "ik_checked":False,"whole_arm_collision_checked":False}
