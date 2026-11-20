"""Map a visual held-body attachment into the existing cabinet planner."""
import copy
import numpy as np
from .docking_fit import OBJECT_TO_TOOL
from .placement_geometry import rigid

def nominal_attachment(contact_mm):
    contact=np.asarray(contact_mm,float)
    if contact.shape!=(3,) or not np.isfinite(contact).all():raise ValueError('Invalid contact')
    T=np.eye(4);T[:3,:3]=OBJECT_TO_TOOL;T[:3,3]=-OBJECT_TO_TOOL@contact/1000
    return T

def effective_tcp(tcp,observed_attachment,contact_mm):
    return rigid(tcp)@rigid(observed_attachment)@np.linalg.inv(nominal_attachment(contact_mm))

def prepared_height(height):
    return np.isfinite(height) and abs(float(height)-3.)<=.3500001

def aperture_recenter(fit):
    """Find a bounded cross-aperture translation with positive nominal gap."""
    points=np.asarray(fit['footprint_uv_mm'],float)
    poly=np.asarray(fit['aperture_polygon_mm'],float)
    if points.ndim!=2 or points.shape[1]!=2 or poly.ndim!=2 or poly.shape[1]!=2 or len(poly)<3 or not np.isfinite(np.r_[points.ravel(),poly.ravel()]).all():
        raise ValueError('Invalid aperture footprint')
    edges=np.roll(poly,-1,axis=0)-poly
    lengths=np.linalg.norm(edges,axis=1)
    area=np.sum(poly[:,0]*np.roll(poly[:,1],-1)-poly[:,1]*np.roll(poly[:,0],-1))
    if np.any(lengths<1e-8) or abs(area)<1e-8:raise ValueError('Invalid aperture polygon')
    normals=np.sign(area)*np.column_stack((-edges[:,1],edges[:,0]))/lengths[:,None]
    baseline=np.min(np.einsum('eij,ej->ei',points[None,:,:]-poly[:,None,:],normals),axis=1)
    grid=np.arange(-5.,5.001,.25)
    candidates=np.array([(x,y) for x in grid for y in grid if x*x+y*y<=25.])
    gaps=np.min(baseline[None,:]+candidates@normals.T,axis=1)
    best=float(np.max(gaps))
    # Prefer the shortest motion among practically equivalent clearances.
    eligible=np.where(gaps>=best-.05)[0]
    idx=eligible[np.argmin(np.linalg.norm(candidates[eligible],axis=1))]
    if gaps[idx]<=.5:raise ValueError('No positive clearance within 5mm correction')
    return candidates[idx],float(gaps[idx])

def scene_allowance(proof,*,inside):
    keys=('aperture','table','arm_standoff') if inside else ('cabinet_standoff','table','arm_standoff')
    slack=min(proof['lower_mm'][k]-proof['required_mm'][k] for k in keys if k in proof['lower_mm'])
    return max(0.,min(.5,slack) if inside else slack)

def updated_bias(previous,nominal,actual):
    return np.clip(np.asarray(previous)+np.asarray(nominal)-np.asarray(actual),-np.radians(2.),np.radians(2.))

def compensate_path(points,bias,limits_deg):
    points=np.asarray(points,float);bias=np.asarray(bias,float);limits=np.radians(limits_deg)
    if points.ndim!=2 or points.shape[1]!=6 or bias.shape!=(6,) or np.max(abs(bias))>np.radians(2.)+1e-12:raise ValueError('Invalid bounded correction')
    out=points+np.linspace(0.,1.,len(points))[:,None]*bias
    if not np.isfinite(out).all() or np.any(out<limits[:,0]) or np.any(out>limits[:,1]):raise ValueError('Correction outside joint limits')
    return out

def retained_reference_path(nominal,held_reference,actual):
    nominal=np.asarray(nominal,float);actual=np.asarray(actual,float);reference=np.asarray(held_reference,float)
    if (nominal.ndim!=2 or nominal.shape[1]!=6 or actual.shape!=(6,) or reference.shape!=(6,)
            or not np.isfinite(np.r_[nominal.ravel(),actual,reference]).all()
            or np.max(abs(nominal[0]-actual))>np.radians(.25)):
        raise ValueError('Invalid retained-reference binding')
    offset=reference-actual
    if np.max(abs(offset))>np.radians(2.):raise ValueError('Measured loaded reference offset exceeds 2deg')
    return nominal+offset

def loaded_trajectory_plan(plan,actual,driver,limits_deg):
    """Anchor a checked joint curve at the reference already supporting the load.

    A constant shift leaves joint velocities/accelerations unchanged. The caller
    must recheck the shifted arm path before dispatch; this is not a mass estimate.
    Already compensated paths receive no second holding offset.
    """
    points=np.asarray(plan['joint_waypoints_rad'],float)
    actual=np.asarray(actual,float);held=np.asarray(driver.get('joint_reference_rad'),float)
    limits=np.radians(np.asarray(limits_deg,float))
    if (points.ndim!=2 or points.shape[1]!=6 or len(points)<2
            or actual.shape!=(6,) or held.shape!=(6,) or limits.shape!=(6,2)
            or not np.isfinite(np.r_[points.ravel(),actual,held,limits.ravel()]).all()):
        raise ValueError('Loaded trajectory reference unavailable or invalid')
    offset=held-points[0]
    if max(np.max(abs(held-actual)),np.max(abs(offset)))>np.radians(2.)+1e-12:
        raise ValueError('Loaded trajectory reference exceeds 2deg bound')
    commands=points+offset;commands[0]=held
    if np.any(commands<limits[:,0]) or np.any(commands>limits[:,1]):
        raise ValueError('Loaded trajectory reference outside joint limits')
    result=copy.deepcopy(plan);result['joint_waypoints_rad']=commands.tolist()
    return result,offset

def capture_motion_bound(captured_tool,current_tool,expected_epoch,current_epoch):
    if current_epoch!=expected_epoch:raise ValueError('Arm command changed during visual exposure processing')
    a,b=rigid(captured_tool),rigid(current_tool)
    angle=np.arccos(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1.,1.))
    # 350mm encloses the entire observed battery around the calibrated tool.
    bound=(np.linalg.norm(b[:3,3]-a[:3,3])+.35*angle)*1000
    if bound>5.:raise ValueError('Body pose changed by more than authorized 5mm processing tolerance')
    return float(bound)
