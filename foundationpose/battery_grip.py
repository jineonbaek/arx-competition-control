"""Pure geometry for the measured battery-handle reference; no motion commands."""
import numpy as np


def transform_reference(profile,base_from_object,camera_from_object):
    base=np.asarray(base_from_object,float)
    camera=np.asarray(camera_from_object,float)
    for T in [base,camera]:
        if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError("Invalid object transform")
        if not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-3) or np.linalg.det(T[:3,:3])<.99:
            raise ValueError("Invalid rigid object transform")
    model=profile["points_object_mm"]
    def points(T):
        return {key:(T[:3,:3]@np.asarray(p)+1000*T[:3,3]).tolist() for key,p in model.items()}
    approach=base[:3,:3]@np.asarray(profile["approach_object"])
    bump=base[:3,:3]@np.asarray(profile["bump_direction_object"])
    approach=approach/np.linalg.norm(approach);bump=bump/np.linalg.norm(bump)
    tool_x=approach
    tool_z=-bump
    tool_y=np.cross(tool_z,tool_x);tool_y/=np.linalg.norm(tool_y)
    tool_z=np.cross(tool_x,tool_y)
    base_from_tool=np.column_stack((tool_x,tool_y,tool_z))
    angle=lambda a,b:float(np.degrees(np.arccos(np.clip(np.dot(a,b),-1,1))))
    return dict(points_object_mm=model,points_base_mm=points(base),points_camera_mm=points(camera),
        insertion_mm=profile["insertion_mm"],required_j6_deg=-90,
        bump_down_error_deg=angle(bump,[0,0,-1]),approach_x_error_deg=angle(approach,[1,0,0]),
        approach_base=approach.tolist(),bump_base=bump.tolist(),R_base_from_tool=base_from_tool.tolist(),
        method="CAD geometric candidate, not a validated motion target")


def draw_reference(profile,base_from_object,camera_from_object,project,vis):
    import cv2
    result=transform_reference(profile,base_from_object,camera_from_object)
    result["profile_version"]=profile["version"]
    result["pixels"]={key:np.asarray(project([np.asarray(point)*.001])[0],int).tolist()
                      for key,point in profile["points_object_mm"].items()}
    points=result["pixels"]
    cv2.arrowedLine(vis,tuple(points["front"]),tuple(points["tip"]),(66,231,236),2,cv2.LINE_AA,tipLength=.15)
    for key,label,color in [("front","F",(255,207,101)),("tip","T",(66,231,236)),("grip","G",(255,146,86))]:
        pixel=tuple(points[key])
        cv2.circle(vis,pixel,5,color,-1,cv2.LINE_AA)
        cv2.putText(vis,label,(pixel[0]+8,pixel[1]-5),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),4,cv2.LINE_AA)
        cv2.putText(vis,label,(pixel[0]+8,pixel[1]-5),cv2.FONT_HERSHEY_SIMPLEX,.6,color,1,cv2.LINE_AA)
    return result
