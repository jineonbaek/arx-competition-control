"""Replay saved camera frames and pose report without robot, ROS, or CAN."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from control import arm_model
from agent.cabinet_preview_geometry import opening
from agent.visual_battery_level import motion_limits
from offline_replay import load_frame
from offline_pipeline import plan_offline

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--d435",required=True);p.add_argument("--pose",required=True);p.add_argument("--d405")
    p.add_argument("--mode",choices=("demo","competition"),default="demo");p.add_argument("--right-offset",type=float,default=0.)
    p.add_argument("--workspace",default="/home/arx/arx-place-hold-20261116");p.add_argument("--control-config",default="/home/arx/arx-competition-control/web/config/control.json")
    a=p.parse_args();root=Path(a.workspace)
    read=lambda path:json.loads(Path(path).read_text())
    frame=load_frame(a.d435);report=read(a.pose);d405=load_frame(a.d405) if a.d405 else None
    sha=hashlib.sha256((root/"web/cad/gripper-v2/battery-print.stl").read_bytes()).hexdigest()
    tcp=np.asarray(read(root/"config/right-grip-tcp.json")["T_wrist_from_tool"],float)
    slot=opening(read(root/"config/cabinet-profiles.json"),a.mode);battery=read(root/"config/battery-envelope.json")
    contact=read(root/"web/public/cad/battery-grip.json")["points_object_mm"]["tip"]
    conf=read(a.control_config);policy=read(root/"config/battery-execution.json")
    result=plan_offline(frame=frame,pose_report=report,mesh_sha=sha,model=arm_model("right"),tcp=tcp,slot=slot,battery=battery,contact=contact,limits=conf["arms"]["right"]["joint_limits_deg"],timing_limits=motion_limits(policy,conf),d405_frame=d405,right_offset_mm=a.right_offset)
    json.dump(result,sys.stdout,ensure_ascii=False,allow_nan=False,indent=2);print()
if __name__=="__main__":
    try:main()
    except (ValueError,KeyError,OSError) as e:
        print(json.dumps({"status":"unavailable","reason":str(e),"executable":False},ensure_ascii=False),file=sys.stderr);sys.exit(2)
