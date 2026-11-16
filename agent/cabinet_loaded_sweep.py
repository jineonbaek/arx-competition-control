"""Nominal RGB/CAD table and loaded-sweep checks; never physical certification."""
import hashlib,json,math,struct
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from .cabinet_rgb_geometry import camera_matrix
from .placement_geometry import rigid


def infer_table(frame,window,profile,slot,mesh_bytes):
    if hashlib.sha256(mesh_bytes).hexdigest()!=profile['sha256'] or len(mesh_bytes)<84:
        raise ValueError('Cabinet mesh identity unavailable for table inference')
    count=struct.unpack_from('<I',mesh_bytes,80)[0]
    if count==0 or len(mesh_bytes)!=84+50*count:raise ValueError('Cabinet STL record count invalid')
    record=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('extra','<u2')])
    vertices=np.frombuffer(mesh_bytes,dtype=record,offset=84)['vertices'].reshape(-1,3)
    bottom=float(vertices[:,profile['plane_axes'][1]].min())
    offset_mm=float(slot['center_mm'][1])-bottom
    if not math.isfinite(offset_mm) or offset_mm<0:raise ValueError('Cabinet floor geometry invalid')
    height=float(window['center_base_m'][2])-float(window['up_axis_base'][2])*offset_mm/1000
    rgb=np.asarray(frame['rgb']);quad=np.asarray(window['inner_quad_px'],float)
    if rgb.dtype!=np.uint8 or rgb.ndim!=3 or quad.shape!=(4,2):raise ValueError('Table color inference unavailable')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask=(hsv[:,:,0]>=8)&(hsv[:,:,0]<=45)&(hsv[:,:,1]>=12)&(hsv[:,:,1]<=140)&(hsv[:,:,2]>100)
    end=min(rgb.shape[0],int(quad[:,1].max()+60));start=max(0,int(quad[:,1].max()-20))
    points=[]
    for x in range(max(40,int(quad[:,0].min()-280)),max(40,min(rgb.shape[1]-20,int(quad[:,0].min()-40)))):
        ys=np.flatnonzero(mask[:end,x])
        if len(ys)<30:continue
        y=int(ys[-1])
        if y<start or y>=end-2 or mask[max(0,y-8):y+1,x].mean()<.6:continue
        points.append([x,y])
    points=np.asarray(points,float)
    if points.ndim!=2 or len(points)<80 or np.ptp(points[:,0])<80:raise ValueError('Visible tabletop front edge insufficient')
    fit=np.polyfit(points[:,0],points[:,1],1)
    good=abs(points[:,1]-np.polyval(fit,points[:,0]))<3.
    if good.mean()<.75:raise ValueError('Tabletop edge ambiguous')
    points=points[good];fit=np.polyfit(points[:,0],points[:,1],1)
    k=frame['intrinsics'];k=json.loads(str(k)) if not isinstance(k,dict) else k
    matrix,_=camera_matrix(k,rgb.shape);T=rigid(frame['base_from_color']);world=[]
    for x in (float(points[:,0].min()),float(points[:,0].max())):
        y=float(np.polyval(fit,x));ray=T[:3,:3]@np.linalg.solve(matrix,[x,y,1.])
        if abs(ray[2])<.05:raise ValueError('Table ray is parallel to inferred plane')
        scale=(height-T[2,3])/ray[2]
        if not .07<scale<3:raise ValueError('Table plane projection out of range')
        world.append(T[:3,3]+scale*ray)
    tangent=np.asarray(world[1])-world[0];normal=np.array([tangent[1],-tangent[0],0.]);normal/=np.linalg.norm(normal)
    if normal@(-np.asarray(window['outward_normal_base']))<0:normal=-normal
    front=float(np.asarray(world).mean(0)@normal)
    gap=float(np.asarray(window['center_base_m'])@normal-front)
    if not 0<gap<1.2:raise ValueError('Table front lies behind cabinet or is inconsistent')
    return dict(status='inferred',nominal_only=True,physical_accuracy_verified=False,
                front_normal_base=normal.tolist(),front_offset_m=front-.020,
                top_m=height+.005,bottom_m=height-.060,edge_support_pixels=len(points),
                front_padding_mm=20.,top_padding_mm=5.,note='RGB edge and CAD floor; not measured environment calibration')


def _root(model,lift):
    chain=np.eye(4)
    if len(model.parts)!=8:raise ValueError('Complete arm geometry unavailable')
    for transform,_,_ in model.parts[:6]:chain=chain@transform
    return model.wrist_transform([0.]*6,lift)@np.linalg.inv(chain)


def _boxes(model,q,lift,root):
    parent=root.copy();wrist=None;boxes=[]
    for i,(m,axis,corners) in enumerate(model.parts):
        link=(parent if i<6 else wrist)@m
        if i<6:
            link[:3,:3]=link[:3,:3]@Rotation.from_rotvec(axis*q[i]).as_matrix();parent=link
        if i==5:wrist=link.copy()
        boxes.append(corners@link[:3,:3].T+link[:3,3])
    if not np.allclose(wrist,model.wrist_transform(q,lift),atol=1e-9,rtol=0):raise ValueError('Arm box chain differs from active FK')
    return boxes


def _gap(a,b):
    delta=np.maximum(np.maximum(a.min(0)-b.max(0),b.min(0)-a.max(0)),0.)
    return float(np.linalg.norm(delta))


def check_outside_sweep(model,left_model,left_q,lift,waypoints,tcp,battery,window,table,*,sample_bound_mm=1.,_table_margin_mm=10.):
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
    minimum={k:math.inf for k in ['cabinet_standoff','table','base','left_arm','own_arm']}
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
            minimum['table']=min(minimum['table'],*(table_gap(points) for points in [payload,*parts]))
            minimum['base']=min(minimum['base'],_gap(payload,base))
            minimum['left_arm']=min(minimum['left_arm'],*(_gap(points,left) for points in [payload,*parts] for left in left_boxes))
            minimum['own_arm']=min(minimum['own_arm'],*(_gap(payload,part) for part in parts[:6]))
    lower={k:v*1000-bound*1000 for k,v in minimum.items()}
    if _table_margin_mm not in (0.,10.):raise ValueError('Unknown table check mode')
    margins=dict(cabinet_standoff=50.,table=_table_margin_mm,base=20.,left_arm=20.,own_arm=10.)
    for name,margin in margins.items():
        if lower[name]<margin:raise ValueError('Loaded outside sweep '+name+' gap %.2fmm below %.1fmm'%(lower[name],margin))
    return dict(clear=True,nominal_only=True,physical_accuracy_verified=False,samples=count,
                between_sample_bound_mm=bound*1000,lower_mm=lower,required_mm=margins,table=table)


def check_translation_sweep(model,left_model,left_q,lift,q,tcp,battery,window,table,delta_m,*,base_moves=False,upward_recovery=False):
    """Conservative union of every intermediate translated box, no sample gaps.

    Lift translates both arms in Z relative to the fixed chassis. A straight
    chassis step translates the whole robot with fixed arm joints; robot-relative
    left/self/base clearances are invariant. Rotation is excluded here.
    """
    d=np.asarray(delta_m,float)
    if (d.shape!=(3,) or not np.isfinite(d).all() or np.linalg.norm(d)>.05
            or (base_moves and abs(d[2])>1e-12) or (not base_moves and np.linalg.norm(d[:2])>1e-12)):
        raise ValueError('Only bounded pure auxiliary translation is proved')
    if upward_recovery and (base_moves or d[2]<=0 or d[2]>.0061):
        raise ValueError('Recovery is at most 6.1mm upward only')
    result=check_outside_sweep(model,left_model,left_q,lift,[q,q],tcp,battery,window,table,_table_margin_mm=0. if upward_recovery else 10.)
    bounds=np.asarray(battery['bounds_mm'],float)/1000;contact=np.asarray(battery['contact_mm'],float)/1000
    obj=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
    rotation=np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
    tool=(obj-contact)@rotation.T;t=model.wrist_transform(q,lift)@tcp
    payload=tool@t[:3,:3].T+t[:3,3];parts=_boxes(model,q,lift,_root(model,lift))
    swept=[np.vstack((p,p+d)) for p in [payload,*parts]]
    normal=np.array(table['front_normal_base']);out=np.array(window['outward_normal_base']);center=np.array(window['center_base_m'])
    lower=dict(result['lower_mm'])
    def table_gap(p):
        return max(0.,float(table['front_offset_m']-(p@normal).max()),float(p[:,2].min()-table['top_m']),float(table['bottom_m']-p[:,2].max()))*1000
    lower['table']=min(table_gap(p) for p in swept)
    lower['cabinet_standoff']=float(((swept[0]-center)@out).min())*1000
    if not base_moves:lower['base']=_gap(swept[0],np.array(model.column))*1000
    if upward_recovery:
        initial=[table_gap(p) for p in [payload,*parts]];end=min(table_gap(p+d) for p in [payload,*parts])
        if min(initial)<=0 or end<10. or any(table_gap(p)<start-1e-7 for p,start in zip(swept,initial)):
            raise ValueError('Upward recovery must never reduce any table gap and must restore 10mm')
    for name,required in result['required_mm'].items():
        if lower[name]<required:raise ValueError('Translated '+name+' gap %.2fmm below %.1fmm'%(lower[name],required))
    answer=dict(result,lower_mm=lower,continuous_geometry='translation_union',translation_m=d.tolist(),samples=2)
    if upward_recovery:
        answer.update(recovery_only=True,end_table_gap_mm=end,required_after_mm=10.,recovery_floor_mm=max(0.,min(initial)-.5))
    return answer
