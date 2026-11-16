"""Recorded-frame adapter for T2 module simulation only; no live camera or motor I/O."""
import hashlib,json
from pathlib import Path
import numpy as np
from control import arm_model
from agent.cabinet_preview_geometry import opening
from agent.visual_battery_level import motion_limits
from offline_replay import load_frame
from offline_pipeline import plan_offline

class RecordedT2Planner:
    def __init__(self,*,frame,pose,workspace,control_config,mode="demo",d405=None,right_offset_mm=0.):
        self.frame=Path(frame);self.pose=Path(pose);self.workspace=Path(workspace);self.control_config=Path(control_config)
        self.mode=mode;self.d405=Path(d405) if d405 else None;self.right_offset_mm=right_offset_mm
    def __call__(self,context):
        if context["stage"]!="place" or context["cancelled"].is_set():raise ValueError("T2 任务已停止或阶段不匹配")
        r=self.workspace;read=lambda path:json.loads(Path(path).read_text())
        frame=load_frame(self.frame);pose=read(self.pose);d405=load_frame(self.d405) if self.d405 else None
        sha=hashlib.sha256((r/"web/cad/gripper-v2/battery-print.stl").read_bytes()).hexdigest()
        tcp=np.asarray(read(r/"config/right-grip-tcp.json")["T_wrist_from_tool"],float)
        slot=opening(read(r/"config/cabinet-profiles.json"),self.mode);battery=read(r/"config/battery-envelope.json")
        contact=read(r/"web/public/cad/battery-grip.json")["points_object_mm"]["tip"]
        conf=read(self.control_config);policy=read(r/"config/battery-execution.json")
        result=plan_offline(frame=frame,pose_report=pose,mesh_sha=sha,model=arm_model("right"),tcp=tcp,slot=slot,battery=battery,contact=contact,limits=conf["arms"]["right"]["joint_limits_deg"],timing_limits=motion_limits(policy,conf),d405_frame=d405,right_offset_mm=self.right_offset_mm)
        if context["cancelled"].is_set():raise ValueError("T2 任务已停止")
        return result
