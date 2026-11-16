"""Full held-object aperture projection. Pure geometry; never motor permission."""
import cv2
import numpy as np
from .placement_geometry import rigid

OBJECT_TO_TOOL=np.array([[0.,0.,-1.],[1.,0.,0.],[0.,-1.,0.]])


def evaluate_fit(tool,window,slot,battery,contact,*,uncertainty_mm=0.,margin_mm=.5):
    """Project every box corner through a fixed-Z insertion axis onto the aperture.

    `tool` is base_from_tool in metres. Object geometry/contact are millimetres.
    `uncertainty_mm` must include relevant calibration, perception and grasp errors;
    zero is useful for nominal model replay, never a claim of physical accuracy.
    """
    tool=rigid(tool)
    for value in (uncertainty_mm,margin_mm):
        if type(value) not in (int,float) or not np.isfinite(value) or value<0:
            raise ValueError('Nonnegative finite clearance allowances required')
    bounds=np.asarray(battery['bounds_mm'],float);contact=np.asarray(contact,float)
    center=np.asarray(window['center_base_m'],float)
    out=np.asarray(window['outward_normal_base'],float)
    left=np.asarray(window['left_axis_base'],float);up=np.asarray(window['up_axis_base'],float)
    basis=np.asarray([out,left,up])
    polygon=np.asarray(slot['outline_mm'],float)-np.asarray(slot['center_mm'],float)
    # CAD X increases toward camera right; window L is camera/robot left.
    polygon[:,0]*=-1
    if (bounds.shape!=(2,3) or contact.shape!=(3,) or center.shape!=(3,) or basis.shape!=(3,3)
            or polygon.ndim!=2 or polygon.shape[1]!=2 or len(polygon)<3
            or not np.isfinite(np.r_[bounds.ravel(),contact,center,basis.ravel(),polygon.ravel()]).all()
            or np.any(bounds[1]<=bounds[0]) or not np.allclose(basis@basis.T,np.eye(3),atol=.02)):
        raise ValueError('Finite box, polygon and orthonormal aperture axes required')
    rounding=0.
    if not cv2.isContourConvex(polygon.astype(np.float32)):
        hull=cv2.convexHull(polygon.astype(np.float32))
        rounding=max(float(cv2.pointPolygonTest(hull,tuple(p),True)) for p in polygon)
        if rounding>.001:raise ValueError('Convex aperture required for a corner-only footprint bound')
    # Never expand a CAD opening when RGB estimates a larger marker.
    size=np.asarray(slot['size_mm'],float)
    measured=np.asarray(window.get('measured_size_mm',window.get('model_aperture_size_mm',size)),float)
    if size.shape!=(2,) or measured.shape!=(2,) or not np.isfinite(np.r_[size,measured]).all() or min(*size,*measured)<=0:
        raise ValueError('Positive aperture dimensions required')
    polygon*=np.minimum(size,measured)/size
    axis=-out.copy();axis[2]=0
    if np.linalg.norm(axis)<.7:raise ValueError('Horizontal insertion is not observable')
    axis/=np.linalg.norm(axis);denominator=float(axis@out)
    if abs(denominator)<.7:raise ValueError('Horizontal insertion is too oblique')
    corners=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
    points=((corners-contact)@OBJECT_TO_TOOL.T/1000)@tool[:3,:3].T+tool[:3,3]
    projected=points-(((points-center)@out)/denominator)[:,None]*axis
    uv=np.column_stack(((projected-center)@left,(projected-center)@up))*1000
    clearances=[float(cv2.pointPolygonTest(polygon.astype(np.float32),tuple(p),True)) for p in uv]
    clearance=min(clearances)-rounding;available=clearance-uncertainty_mm-margin_mm
    return dict(scope='aperture_projection_only',nominal_fit=clearance>0,qualified_fit=available>0,
                nominal_clearance_mm=clearance,available_clearance_mm=available,
                uncertainty_mm=uncertainty_mm,margin_mm=margin_mm,
                footprint_uv_mm=uv.tolist(),aperture_polygon_mm=polygon.tolist(),
                leading_standoff_mm=float(np.min((points-center)@out)*1000),
                tilt_deg=float(np.degrees(np.arcsin(np.clip(tool[2,0],-1,1)))),
                nominal_only=uncertainty_mm==0,physical_accuracy_verified=False)
