"""Offline analysis of archived JPEG + sparse base-frame cloud. Never controls hardware."""
import json
from pathlib import Path
import cv2
import numpy as np
from d405_partial import estimate_vertical_alignment

INTRINSICS=json.loads((Path(__file__).resolve().parent/'intrinsics.json').read_text())

def red_mask(image):
    if not isinstance(image,np.ndarray) or image.ndim!=3 or image.shape[2]!=3 or image.dtype!=np.uint8:
        raise ValueError('invalid BGR image')
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)).astype(np.uint8)*255
    return cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))

def detect_cabinet(image):
    mask=red_mask(image)
    contours,hier=cv2.findContours(mask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
    choices=[]
    if hier is not None:
        for i,c in enumerate(contours):
            if hier[0,i,3]!=-1 or cv2.contourArea(c)<300:continue
            child=hier[0,i,2]
            if child<0:continue
            outer=float(cv2.contourArea(c));inner=contours[child];ratio=cv2.contourArea(inner)/outer
            x,y,w,h=cv2.boundingRect(inner)
            if .30<ratio<.97 and min(w,h)>=15:choices.append((outer,(x,y,w,h)))
    if len(choices)!=1:raise ValueError('red opening missing or ambiguous')
    _,(x,y,w,h)=choices[0]
    return dict(status='detected',center_px=[x+w/2.,y+h/2.],opening_bbox_px=[x,y,x+w,y+h],executable=False)

def project_points(view,intrinsics):
    points=np.asarray(view['points'],float);transform=np.asarray(view['base_from_color'],float)
    k=np.asarray([intrinsics[name] for name in ('fx','fy','cx','cy')],float)
    if not np.isfinite(k).all() or min(k[:2])<=0:raise ValueError('invalid camera intrinsics')
    if points.ndim!=2 or points.shape[1]<3 or transform.shape!=(4,4) or not np.isfinite(transform).all():
        raise ValueError('invalid point cloud or transform')
    if not np.allclose(transform[3],[0,0,0,1]) or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-5):
        raise ValueError('non-rigid camera transform')
    cam=(points[:,:3]-transform[:3,3])@transform[:3,:3]
    good=np.isfinite(cam).all(1)&(cam[:,2]>.01)
    cam=cam[good];base=points[good,:3]
    uv=np.column_stack((intrinsics['fx']*cam[:,0]/cam[:,2]+intrinsics['cx'],intrinsics['fy']*cam[:,1]/cam[:,2]+intrinsics['cy']))
    good=np.isfinite(uv).all(1)&(uv[:,0]>=0)&(uv[:,0]<view['width']-.5)&(uv[:,1]>=0)&(uv[:,1]<view['height']-.5)
    return dict(camera_xyz=cam[good],base_xyz=base[good],pixel_xy=np.rint(uv[good]).astype(int))

def reconstruct_sparse(view,intrinsics):
    proj=project_points(view,intrinsics);h,w=int(view['height']),int(view['width']);xyz=np.zeros((h,w,3),np.float32)
    dist=np.full((h,w),np.inf,np.float32);uv=proj['pixel_xy'];cam=proj['camera_xyz']
    # Fill only near measured points. Archived clouds have 4-pixel sampling;
    # projection ignores inverse-Brown distortion and is not calibrated depth.
    for dx in range(-3,4):
        for dy in range(-3,4):
            d=dx*dx+dy*dy
            if d>10:continue
            xx=uv[:,0]+dx;yy=uv[:,1]+dy;valid=(xx>=0)&(xx<w)&(yy>=0)&(yy<h)
            xx=xx[valid];yy=yy[valid];p=cam[valid];better=d<dist[yy,xx]
            xyz[yy[better],xx[better]]=p[better];dist[yy[better],xx[better]]=d
    return xyz,proj

def image_from_view(view):
    import base64
    uri=view.get('image','')
    if not uri.startswith('data:image/jpeg;base64,'):raise ValueError('archived JPEG missing')
    image=cv2.imdecode(np.frombuffer(base64.b64decode(uri.split(',',1)[1]),np.uint8),cv2.IMREAD_COLOR)
    if image is None or image.shape[:2]!=(view['height'],view['width']):raise ValueError('archived JPEG dimensions invalid')
    return image

def coarse_3d(view,image,detection,intrinsics):
    proj=project_points(view,intrinsics);uv=proj['pixel_xy'];base=proj['base_xyz']
    mask=red_mask(image);x0,y0,x1,y1=detection['opening_bbox_px'];pad=18
    uv[:,0]=np.clip(uv[:,0],0,image.shape[1]-1);uv[:,1]=np.clip(uv[:,1],0,image.shape[0]-1)
    selected=(mask[uv[:,1],uv[:,0]]>0)&(uv[:,0]>=x0-pad)&(uv[:,0]<=x1+pad)&(uv[:,1]>=y0-pad)&(uv[:,1]<=y1+pad)
    points=base[selected]
    if len(points)<30:raise ValueError('insufficient red rim point cloud')
    rng=np.random.default_rng(0);best=None
    for _ in range(160):
        a,b,c=points[rng.choice(len(points),3,replace=False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-8:continue
        n/=length;inside=np.abs((points-a)@n)<.008
        if best is None or inside.sum()>best.sum():best=inside
    if best is None or best.sum()<30:raise ValueError('rim plane not sufficiently supported')
    support=points[best];center=support.mean(0);_,_,vh=np.linalg.svd(support-center,full_matrices=False);normal=vh[-1];T=np.asarray(view['base_from_color'],float)
    if normal@(T[:3,3]-center)<0:normal=-normal
    k=intrinsics;u,v=detection['center_px'];ray=T[:3,:3]@np.array([(u-k['cx'])/k['fx'],(v-k['cy'])/k['fy'],1.]);den=float(normal@ray)
    if abs(den)<.1:raise ValueError('rim plane nearly parallel to center ray')
    distance=float((normal@(center-T[:3,3]))/den)
    if not .07<distance<3:raise ValueError('rim distance invalid')
    target=T[:3,3]+distance*ray;res=np.abs((support-center)@normal)*1000
    return dict(center_base_m=target.tolist(),outward_normal_base=normal.tolist(),approach_axis_base=(-normal).tolist(),rim_points=int(len(support)),plane_p90_mm=float(np.percentile(res,90)),method='sparse_cloud_approximate_projection',executable=False)

def analyze_saved(d435,d405,*,intrinsics=None):
    intrinsics=INTRINSICS if intrinsics is None else intrinsics
    a=json.loads(Path(d435).read_text());b=json.loads(Path(d405).read_text());image435=image_from_view(a);image405=image_from_view(b)
    found=detect_cabinet(image435);found['coarse']=coarse_3d(a,image435,found,intrinsics['d435'])
    xyz,_=reconstruct_sparse(b,intrinsics['d405']);frame=dict(camera_id='d405',rgb=cv2.cvtColor(image405,cv2.COLOR_BGR2RGB),xyz=xyz,base_from_color=b['base_from_color'])
    try:
        fine=estimate_vertical_alignment(frame);fine['source']='reprojected_sparse_cloud'
    except (ValueError,KeyError) as error:fine=dict(status='unavailable',reason=str(error),executable=False)
    delta=abs(float(a['time'])-float(b['time']))
    return dict(d435=found,d405=fine,camera_timestamp_delta_s=delta,time_paired=bool(delta<=.2),executable=False,
        limitations=['archived images and 4-pixel-sampled point clouds only','inverse-Brown distortion ignored during reprojection','camera extrinsics and physical accuracy unverified','no freshness, motion, path collision, or complete insertion validation'])

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('d435');parser.add_argument('d405');parser.add_argument('--intrinsics');args=parser.parse_args()
    intrinsics=json.loads(Path(args.intrinsics).read_text()) if args.intrinsics else INTRINSICS
    print(json.dumps(analyze_saved(args.d435,args.d405,intrinsics=intrinsics),ensure_ascii=False,indent=2,allow_nan=False))
