"""Held-body insertion sweep with an explicit positive table clearance."""
import numpy as np,math
from agent.placement_geometry import rigid
from agent.docking_fit import OBJECT_TO_TOOL
from agent.shallow_aperture import evaluate_shallow_fit as evaluate_fit
from agent.shallow_aperture import aperture_relevant
from agent.cabinet_loaded_sweep import _boxes,_root,_gap

def validate_clearances(lower,required,*,skip_table=False):
    for name,margin in required.items():
        if name=='table' and skip_table:continue
        if lower[name]<margin:raise ValueError(name+' nominal gap %.3fmm below %.3fmm'%(lower[name],margin))

def _insert_sweep(ctrl,state,points,tcp,battery,contact,window,slot,table,*,table_margin_mm=10.,measured=False,skip_table=False):
    """Nominal complete payload/arm/table sweep; aperture uncertainty is reported separately."""
    model=ctrl.models['right'];left=ctrl.models['left'];lift=state['body']['height']
    left_boxes=_boxes(left,state['arms']['left']['joints'],lift,_root(left,lift));root=_root(model,lift)
    bounds=np.asarray(battery['bounds_mm'],float);corners=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
    payload=(corners-contact)@OBJECT_TO_TOOL.T/1000
    center=np.asarray(window['center_base_m']);out=np.asarray(window['outward_normal_base'])
    normal=np.asarray(table['front_normal_base']);radius=model.reach+np.linalg.norm(tcp[:3,3])+np.max(np.linalg.norm(payload,axis=1))
    lower={k:float('inf') for k in ('aperture','table','arm_standoff','left_arm','base','own_arm')};count=0
    bound=0. if np.all(np.asarray(points)==np.asarray(points)[0]) else .00005
    for a,b in zip(np.asarray(points),np.asarray(points)[1:]):
        steps=max(1,int(np.ceil(radius*np.abs(b-a).sum()/(2*bound)))) if bound else 1
        for f in np.linspace(0,1,steps+1):
            q=a+f*(b-a);T=model.wrist_transform(q,lift)@tcp;box=payload@T[:3,:3].T+T[:3,3]
            parts=_boxes(model,q,lift,root);count+=1
            fit=evaluate_fit(T,window,slot,battery,contact,depth_mm=7. if measured else 5.)
            if not measured or aperture_relevant(fit['leading_standoff_mm']):
                lower['aperture']=min(lower['aperture'],fit['nominal_clearance_mm']-bound*1000)
            lower['arm_standoff']=min(lower['arm_standoff'],min(float(((p-center)@out).min())*1000 for p in parts)-bound*1000)
            for p in [box,*parts]:
                gap=max(0.,float(table['front_offset_m']-(p@normal).max()),float(p[:,2].min()-table['top_m']),float(table['bottom_m']-p[:,2].max()))
                lower['table']=min(lower['table'],gap*1000-bound*1000)
                lower['left_arm']=min(lower['left_arm'],*(_gap(p,l)*1000-bound*1000 for l in left_boxes))
            lower['base']=min(lower['base'],_gap(box,np.array(model.column))*1000-bound*1000)
            lower['own_arm']=min(lower['own_arm'],*(_gap(box,p)*1000-bound*1000 for p in parts[:6]))
    if not .5<=table_margin_mm<=10.:raise ValueError('Positive table clearance required')
    required=dict(aperture=.5,table=float(table_margin_mm),arm_standoff=20.,left_arm=20.,base=20.,own_arm=10.)
    if not count:raise ValueError('No insertion sweep samples')
    validate_clearances(lower,required,skip_table=skip_table)
    if skip_table:lower.pop('table');required.pop('table')
    return dict(clear=True,nominal_only=True,physical_accuracy_verified=False,samples=count,lower_mm=lower,required_mm=required,table_checked=not skip_table)

def valid_outside_margin(margin):
    return bool(np.isfinite(margin) and .1<=margin<=50.)

def check_outside_sweep(model,left_model,left_q,lift,waypoints,tcp,battery,window,table,*,sample_bound_mm=1.,_table_margin_mm=10.,cabinet_margin_mm=50.,skip_table=False):
    if not valid_outside_margin(cabinet_margin_mm):
        raise ValueError('Positive outside cabinet clearance required')
    if table.get('status')!='inferred':raise ValueError('Tabletop boundary unavailable; no loaded outside motion')
    q=np.asarray(waypoints,float);tcp=rigid(tcp)
    if q.ndim!=2 or q.shape[1]!=6 or len(q)<2 or not np.isfinite(q).all():raise ValueError('Loaded sweep waypoints invalid')
    bounds=np.asarray(battery['bounds_mm'],float)/1000;contact=np.asarray(battery['contact_mm'],float)/1000
    obj=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
    rotation=np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
    tool_points=(obj-contact)@rotation.T
    normal=np.asarray(table['front_normal_base']);out=np.asarray(window['outward_normal_base']);center=np.asarray(window['center_base_m'])
    if (not np.isfinite(np.r_[normal,out,center]).all() or abs(np.linalg.norm(normal)-1)>.001
            or abs(normal[2])>.001 or abs(np.linalg.norm(out)-1)>.001):raise ValueError('Loaded scene axes invalid')
    base=np.array([model.column[0],model.column[1]])
    root=_root(model,lift);left_boxes=_boxes(left_model,left_q,lift,_root(left_model,lift))
    minimum={k:math.inf for k in ['cabinet_standoff','arm_standoff','table','base','left_arm','own_arm']}
    if not np.isfinite(sample_bound_mm) or not .1<=sample_bound_mm<=1.:
        raise ValueError('Continuous sampling bound must be 0.1..1mm')
    count=0;sampling_bound=float(sample_bound_mm)/1000
    bound=0. if np.all(q==q[0]) else sampling_bound
    radius=model.reach+np.linalg.norm(tcp[:3,3])+np.linalg.norm(tool_points,axis=1).max()
    def table_gap(points):
        return max(0.,float(table['front_offset_m']-(points@normal).max()),
                   float(points[:,2].min()-table['top_m']),float(table['bottom_m']-points[:,2].max()))
    for a,b in zip(q,q[1:]):
        steps=max(1,int(math.ceil(radius*np.abs(b-a).sum()/(2*sampling_bound))))
        for fraction in np.linspace(0,1,steps+1):
            angles=a+(b-a)*fraction;t=model.wrist_transform(angles,lift)@tcp
            payload=tool_points@t[:3,:3].T+t[:3,3];parts=_boxes(model,angles,lift,root);count+=1
            minimum['cabinet_standoff']=min(minimum['cabinet_standoff'],float(((payload-center)@out).min()))
            minimum['arm_standoff']=min(minimum['arm_standoff'],*(float(((part-center)@out).min()) for part in parts))
            minimum['table']=min(minimum['table'],*(table_gap(points) for points in [payload,*parts]))
            minimum['base']=min(minimum['base'],_gap(payload,base))
            minimum['left_arm']=min(minimum['left_arm'],*(_gap(points,left) for points in [payload,*parts] for left in left_boxes))
            minimum['own_arm']=min(minimum['own_arm'],*(_gap(payload,part) for part in parts[:6]))
    lower={k:v*1000-bound*1000 for k,v in minimum.items()}
    if _table_margin_mm!=0. and not .5<=_table_margin_mm<=10.:raise ValueError('Unknown table check mode')
    margins=dict(cabinet_standoff=float(cabinet_margin_mm),arm_standoff=20.,table=_table_margin_mm,base=20.,left_arm=20.,own_arm=10.)
    validate_clearances(lower,margins,skip_table=skip_table)
    if skip_table:lower.pop('table');margins.pop('table')
    return dict(clear=True,nominal_only=True,physical_accuracy_verified=False,samples=count,
                between_sample_bound_mm=bound*1000,lower_mm=lower,required_mm=margins,table=table,table_checked=not skip_table)
