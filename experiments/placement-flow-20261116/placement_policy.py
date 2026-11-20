"""Offline placement intent and sensor selection; never sends motion commands."""
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class PlacementPolicy:
    phases: tuple = ("prepare", "scan_plan", "dual_camera_align", "insert_5mm", "hold")
    insertion_mm: float = 5.0
    requires_table: bool = False
    gripper_action: str = "retain"
    grip_and_level_result: str = "advisory"

def right_offset_mm(left_reference_uv_mm, measured_uv_mm):
    """Positive U points toward robot left; allow reference through 8 mm right."""
    if not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in (left_reference_uv_mm,measured_uv_mm)):
        raise ValueError("finite lateral coordinates required")
    offset=left_reference_uv_mm-measured_uv_mm
    if offset < -1e-9 or offset > 8.0+1e-9:
        raise ValueError("outside permitted left reference to 8mm right interval")
    return max(0.,min(8.,offset))

def choose_alignment(d405,d435,axis="vertical_offset"):
    """Choose measured quantity by camera priority; never confuse height and angle."""
    if axis not in ("vertical_offset","tilt"):
        raise ValueError("unsupported quantity")
    for camera,observation in (("d405",d405),("d435",d435)):
        if not isinstance(observation,dict): continue
        value=observation.get("value")
        if (observation.get("camera")==camera and observation.get("valid") is True
                and observation.get("axis")==axis and isinstance(value,(int,float))
                and not isinstance(value,bool) and math.isfinite(value)):
            return observation
    return None

def replay_offline(callbacks):
    """Exercise stage order in simulation only, without a robot adapter."""
    if set(callbacks)!=set(PlacementPolicy().phases):
        raise ValueError("all offline callbacks required")
    results={}
    for phase in PlacementPolicy().phases:
        results[phase]=callbacks[phase]()
    return results

def evaluate_offline(scene):
    """Summarize recorded observations; does not authorize or issue movement."""
    offset=right_offset_mm(scene["left_reference_uv_mm"],scene["measured_uv_mm"])
    d405=scene.get("d405");d435=scene.get("d435")
    vertical=choose_alignment(d405,d435,"vertical_offset")
    tilt=choose_alignment(d405,d435,"tilt")
    advisories=[]
    if scene.get("grip")!="stable":advisories.append("grip_unknown")
    if vertical is None:advisories.append("vertical_unavailable")
    if tilt is None:advisories.append("tilt_unavailable")
    return dict(phases=PlacementPolicy().phases,right_offset_mm=offset,
                vertical=vertical,tilt=tilt,advisories=advisories,
                insertion_mm=5.,gripper_action="retain",executable=False)

if __name__=="__main__":
    import json,sys
    try:
        result=evaluate_offline(json.load(sys.stdin))
        json.dump(result,sys.stdout,ensure_ascii=False,allow_nan=False,indent=2)
        print()
    except (ValueError,TypeError,KeyError) as error:
        print(json.dumps({"error":str(error),"executable":False},ensure_ascii=False),file=sys.stderr)
        sys.exit(2)
