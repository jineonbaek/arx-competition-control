import json,cv2,numpy as np,sys
from pathlib import Path
from camera_geometry import camera_intrinsics
from battery_edges import detect_face,fit_line
from agent.cabinet_preview_geometry import opening
p=Path(sys.argv[1]);f=dict(np.load(p/'d405.npz'));rgb=f['rgb'];xyz=f['xyz'];T=f['base_from_color'];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);red=(((hsv[:,:,0]<12)|(hsv[:,:,0]>168))&(hsv[:,:,1]>90)&(hsv[:,:,2]>60)).astype(np.uint8);n,labels,stats,cent=cv2.connectedComponentsWithStats(red,8);i=max(range(1,n),key=lambda i:stats[i,4]);red=(labels==i).astype(np.uint8);x,y,w,h,_=stats[i]
rs,intr=camera_intrinsics(json.loads(str(f['intrinsics'])));ray=lambda uv:np.asarray(rs.rs2_deproject_pixel_to_point(intr,list(map(float,uv)),1.));rng=np.random.default_rng(9)
def plane(mask):
 good=(mask>0)&np.isfinite(xyz).all(axis=2)&(xyz[:,:,2]>.1)&(xyz[:,:,2]<.8);pts=xyz[good];pts=pts[::max(1,len(pts)//3000)];best=np.zeros(len(pts),bool)
 for _ in range(120):
  a,b,c=pts[rng.choice(len(pts),3,False)];normal=np.cross(b-a,c-a);ln=np.linalg.norm(normal)
  if ln<1e-8:continue
  normal/=ln;inside=abs((pts-a)@normal)<.002
  if inside.sum()>best.sum():best=inside
 pts=pts[best];center=pts.mean(0);_,_,v=np.linalg.svd(pts-center,full_matrices=False);n=v[-1]
 if n@center>0:n=-n
 d=-n@center
 return n,d,dict(support=len(pts),p90_mm=float(np.percentile(abs(pts@n+d),90)*1000))
rn,rd,rstats=plane(cv2.erode(red,np.ones((3,3),np.uint8)))
q=np.asarray(detect_face(rgb)['quad_px']);bm=np.zeros(red.shape,np.uint8);cv2.fillConvexPoly(bm,np.rint(q).astype(int),1);bn,bd,bstats=plane(cv2.erode(bm,np.ones((9,9),np.uint8)))
def point(uv,n,d):
 r=ray(uv);return r*(-d/(n@r))
def world(v):return T[:3,:3]@v+T[:3,3]
points=[]
for yy in range(y+int(.2*h),y+h):
 xs=np.flatnonzero(red[yy]);runs=np.split(xs,np.flatnonzero(np.diff(xs)>1)+1) if len(xs) else []
 for run in runs:
  xx=int(run[0]);neighbor=rgb[yy,max(0,xx-4):xx]
  if len(run)>=5 and xx>x+.55*w and len(neighbor) and neighbor.mean()>150 and np.min(neighbor.mean(axis=0))>110:points.append([xx,yy])
print('inner_upper_support',len(points));line,rms=fit_line(points);print('upper line',line,'rms',rms)
ends=[point(z,bn,bd) for z in q];worldends=np.array([world(z) for z in ends]);pedge=np.mean(points,axis=0);upper=world(point(pedge,rn,rd));battery_top=worldends[:2,2].max()
print('rim_plane',rstats,'battery_plane',bstats,'face_edge_lengths_mm',[float(np.linalg.norm(ends[(i+1)%4]-ends[i])*1000) for i in range(4)])
print('world_corners',worldends.tolist());print('upper',upper.tolist(),'upper_minus_batterytop_mm',(upper[2]-battery_top)*1000,'plane_norm_world',(T[:3,:3]@rn).tolist())
slot=opening(json.loads(Path('/home/arx/arx-place-hold-20261116/config/cabinet-profiles.json').read_text()),'demo');print('slot_size',slot['size_mm'])
(p/'d405-plane-diagnostic.json').write_text(json.dumps(dict(rim_plane_n=rn.tolist(),rim_plane_d=float(rd),rim_quality=rstats,battery_plane_n=bn.tolist(),battery_plane_d=float(bd),battery_quality=bstats,battery_corners_base=worldends.tolist(),upper_edge_base=upper.tolist(),upper_minus_battery_top_mm=float((upper[2]-battery_top)*1000),inner_upper_pixels=points,plane_norm_world=(T[:3,:3]@rn).tolist())))
