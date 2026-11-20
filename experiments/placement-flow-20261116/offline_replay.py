"""Recorded RGB-D perception replay only; contains no robot interfaces."""
import numpy as np
from agent.cabinet_preview_geometry import localize
from agent.d405_aperture_alignment import estimate_vertical_alignment
from placement_policy import PlacementPolicy

def replay_frames(d435_frame,d405_frame):
    if str(d435_frame.get("camera_id"))!="d435" or str(d405_frame.get("camera_id"))!="d405":
        raise ValueError("D435 and D405 frames required in order")
    cabinet=localize(d435_frame,now=float(d435_frame["image_time"]))
    vertical=estimate_vertical_alignment(d405_frame)
    return {"mode":"offline_replay","phases":PlacementPolicy().phases,"cabinet":cabinet,
            "vertical":vertical,"tilt":None,"tilt_reason":"D405 estimates position, not battery angle",
            "insertion_mm":5.,"gripper_action":"retain","executable":False}

def load_frame(path):
    with np.load(path,allow_pickle=False) as data:
        return {key:data[key] for key in data.files}


def preview_shallow(window,slot,battery,contact,*,right_offset_mm=0.):
    import cv2
    from agent.cabinet_preview_geometry import horizontal_plan
    from placement_policy import right_offset_mm as check_offset
    offset=check_offset(0.,-right_offset_mm)
    # The geometry planner subtracts the cross-section normal extent; its depth
    # limit is not the resulting leading-edge travel for a tilted opening.
    lo,hi=5.,15.
    for _ in range(16):
        request=(lo+hi)/2.
        plan=horizontal_plan(window,slot,battery,tip_point_object_mm=contact,
                             preclearance_mm=5.,max_insertion_depth_mm=request)
        if plan["leading_insertion_depth_mm"]<5.:lo=request
        else:hi=request
    plan=horizontal_plan(window,slot,battery,tip_point_object_mm=contact,
                         preclearance_mm=5.,max_insertion_depth_mm=lo)
    if abs(plan["leading_insertion_depth_mm"]-5.)>.01:
        raise ValueError("cannot construct the requested nominal 5mm travel")
    outline=np.asarray(slot["outline_mm"],np.float32)-np.asarray(slot["center_mm"],np.float32)
    outline[:,0]*=-1
    outline*=np.asarray(plan["effective_opening_size_mm"],np.float32)/np.asarray(slot["size_mm"],np.float32)
    samples=np.asarray(plan["aperture_envelope_samples_mm"],float)
    samples[:,0]-=offset
    clearance=min(float(cv2.pointPolygonTest(outline,tuple(point),True)) for point in samples)
    if clearance<=0:raise ValueError("requested right offset crosses nominal aperture")
    shift=-np.asarray(window["left_axis_base"],float)*offset
    plan["tcp_points_base_mm"]=(np.asarray(plan["tcp_points_base_mm"],float)+shift).tolist()
    for key in ("leading_points_base_mm","tcp_pre_base_mm","tcp_end_base_mm"):
        plan[key]=(np.asarray(plan[key],float)+shift).tolist()
    plan["aperture_envelope_samples_mm"]=samples.tolist()
    plan["nominal_cad_clearance_mm"]=clearance
    plan["right_offset_mm"]=offset
    return plan

if __name__=="__main__":
    import argparse,json,sys
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("d435_npz");parser.add_argument("d405_npz")
    args=parser.parse_args()
    try:print(json.dumps(replay_frames(load_frame(args.d435_npz),load_frame(args.d405_npz)),ensure_ascii=False,allow_nan=False,indent=2))
    except (ValueError,KeyError,OSError) as error:
        print(json.dumps({"error":str(error),"executable":False},ensure_ascii=False),file=sys.stderr);sys.exit(2)
