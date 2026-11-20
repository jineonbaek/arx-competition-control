"""Replay saved stationary D405 RGB-D frames. No ROS, CAN or robot commands."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from control import arm_model
from d405_axis import axis_consensus
from d405_level import level_from_axis

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("frames",nargs="+");p.add_argument("--config",default="/home/arx/arx-competition-control/web/config/control.json");args=p.parse_args()
    frames=[]
    for name in args.frames:
        with np.load(name,allow_pickle=False) as f:frames.append({k:f[k] for k in f.files})
    consensus=axis_consensus(frames);last=frames[-1];config=json.loads(Path(args.config).read_text())
    proposal=level_from_axis(consensus,arm_model("right"),last["joints"],float(last["lift_rad"]),config["arms"]["right"]["joint_limits_deg"])
    json.dump({"consensus":consensus,"proposal":proposal,"executable":False},sys.stdout,ensure_ascii=False,indent=2,allow_nan=False);print()
if __name__=="__main__":
    try:main()
    except (ValueError,KeyError,OSError) as e:print(json.dumps({"error":str(e),"executable":False},ensure_ascii=False),file=sys.stderr);sys.exit(2)
