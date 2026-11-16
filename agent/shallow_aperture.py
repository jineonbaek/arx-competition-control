"""Aperture occupancy of only the battery volume entering a bounded shallow trial."""
import cv2,numpy as np
from .docking_fit import evaluate_fit,OBJECT_TO_TOOL

def aperture_relevant(leading_standoff_mm):
    if not np.isfinite(leading_standoff_mm):raise ValueError('Invalid measured entrance distance')
    return leading_standoff_mm<=5.

def evaluate_shallow_fit(tool,window,slot,battery,contact,*,depth_mm=5.,uncertainty_mm=0.,margin_mm=.5):
    full=evaluate_fit(tool,window,slot,battery,contact,uncertainty_mm=uncertainty_mm,margin_mm=margin_mm)
    if not np.isfinite(depth_mm) or not 0<depth_mm<=7.:raise ValueError('Shallow trial bounded to 7mm')
    bounds=np.asarray(battery['bounds_mm'],float);contact=np.asarray(contact,float);tool=np.asarray(tool,float)
    out=np.asarray(window['outward_normal_base']);center=np.asarray(window['center_base_m'])
    axis=-out.copy();axis[2]=0.;axis/=np.linalg.norm(axis);den=float(axis@out)
    corners=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
    points=((corners-contact)@OBJECT_TO_TOOL.T/1000)@tool[:3,:3].T+tool[:3,3]
    distance=(full['leading_standoff_mm']+depth_mm)/1000/(-den)
    if distance<-.00001:raise ValueError('Battery beyond bounded shallow depth')
    end=points+axis*max(0.,distance);signed=(end-center)@out
    cap=[p for p,d in zip(end,signed) if d<=1e-10]
    for i in range(8):
        for j in range(i+1,8):
            if (i^j) not in (1,2,4) or signed[i]*signed[j]>=0:continue
            cap.append(end[i]+signed[i]/(signed[i]-signed[j])*(end[j]-end[i]))
    cap=np.array(cap)
    if len(cap)<3:raise ValueError('Shallow entering volume unavailable')
    projected=cap-(((cap-center)@out)/den)[:,None]*axis
    uv=np.column_stack(((projected-center)@np.asarray(window['left_axis_base']),(projected-center)@np.asarray(window['up_axis_base'])))*1000
    polygon=np.array(full['aperture_polygon_mm'],np.float32)
    clearance=min(float(cv2.pointPolygonTest(polygon,tuple(p),True)) for p in uv)
    available=clearance-uncertainty_mm-margin_mm
    return full|dict(scope='finite_shallow_entering_volume',depth_mm=float(depth_mm),
        nominal_fit=clearance>0,qualified_fit=available>0,nominal_clearance_mm=clearance,
        available_clearance_mm=available,footprint_uv_mm=uv.tolist(),entering_vertices=len(cap),
        full_length_projection_clearance_mm=full['nominal_clearance_mm'])
