import cv2,numpy as np
from overlay import arrow,label

def direction(img,result,rgb,depth,meta):
 if result.get('rim',{}).get('status')!='four_edges':return
 intr=meta['depth']['intrinsics'];fx,fy,cx,cy=[float(intr[k]) for k in ('fx','fy','cx','cy')]
 if any(abs(float(v))>1e-8 for v in intr.get('coeffs',[])):return
 if depth.shape!=rgb.shape[:2]:return
 q=np.array(result['rim']['quad_px']);hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);yy,xx=np.indices(depth.shape)
 red=((hsv[:,:,0]<12)|(hsv[:,:,0]>168))&(hsv[:,:,1]>90)&(hsv[:,:,2]>60)
 roi=(xx>=q[:,0].min()-22)&(xx<=q[:,0].max()+22)&(yy>=q[:,1].min()-22)&(yy<=q[:,1].max()+22)
 z=depth.astype(float)*meta['depth']['scale_m_per_unit'];good=red&roi&(z>.1)&(z<2.)
 points=np.column_stack(((xx[good]-cx)*z[good]/fx,(yy[good]-cy)*z[good]/fy,z[good]))
 if len(points)<150:return
 points=points[::max(1,len(points)//3000)];rng=np.random.default_rng(7);best=None
 for _ in range(70):
  a,b,c=points[rng.choice(len(points),3,replace=False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
  if length<1e-8:continue
  n/=length
  if abs(n[2])<.4:continue
  mask=abs((points-a)@n)<.004
  if best is None or mask.sum()>best.sum():best=mask
 if best is None or best.sum()<150 or best.mean()<.5:return
 p=points[best];center=p.mean(0);_,sv,v=np.linalg.svd(p-center,full_matrices=False)
 if sv[1]<.02:return
 n=v[-1];n*=1 if n[2]>0 else -1
 end=center+.07*n
 project=lambda p:np.array([fx*p[0]/p[2]+cx,fy*p[1]/p[2]+cy])
 a=project(center);b=project(end);arrow(img,a,b,(255,90,70));label(img,'IN (depth estimate)',b+[6,8],(255,90,70))
 tilt=float(np.degrees(np.arccos(np.clip(n[2],-1,1))))
 label(img,'face/camera: %.1f deg'%tilt,(12,img.shape[0]-18),(255,90,70))
 result['orientation']=dict(source='red_rim_depth_plane',normal_into_camera=n.tolist(),camera_frame='x right, y down, z away',face_camera_angle_deg=tilt,support_points=len(p),physical_accuracy_verified=False)
