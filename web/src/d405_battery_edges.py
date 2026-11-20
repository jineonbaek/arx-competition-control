"""Four-line perspective fit of the visible battery face; pixel evidence only.
No robot/control imports. Homography is not camera extrinsic or metric calibration.
"""
from collections import deque
import cv2,numpy as np

def fit_line(points):
 p=np.asarray(points,np.float32)
 if len(p)<12:raise ValueError('Insufficient line support')
 vx,vy,x,y=cv2.fitLine(p,cv2.DIST_HUBER,0,.01,.01).ravel();l=np.array([-vy,vx,vy*x-vx*y],float)
 errors=abs(np.c_[p,np.ones(len(p))]@l);keep=errors<3.
 if keep.mean()<.55 or keep.sum()<12:raise ValueError('Line support inconsistent')
 vx,vy,x,y=cv2.fitLine(p[keep],cv2.DIST_L2,0,.01,.01).ravel();l=np.array([-vy,vx,vy*x-vx*y],float)
 return l,float(np.sqrt(np.mean((np.c_[p[keep],np.ones(keep.sum())]@l)**2)))

def intersection(a,b):
 p=np.cross(a,b)
 if abs(p[2])<1e-8:raise ValueError('Adjacent edges parallel')
 return p[:2]/p[2]

def projective_geometry(quad):
 q=np.asarray(quad,np.float32)
 if q.shape!=(4,2) or not np.isfinite(q).all() or not cv2.isContourConvex(q) or cv2.contourArea(q)<100:raise ValueError('Invalid convex face')
 H=cv2.getPerspectiveTransform(np.array([[0,0],[1,0],[1,1],[0,1]],np.float32),q)
 axis=cv2.perspectiveTransform(np.array([[[.5,0],[.5,1],[.5,.5]]],np.float32),H)[0]
 edges=[np.cross(np.r_[q[i],1],np.r_[q[(i+1)%4],1]) for i in range(4)]
 vanishing=[]
 for i,j in ((0,2),(1,3)):
  v=np.cross(edges[i],edges[j]);v=v/np.linalg.norm(v)
  vanishing.append(v.tolist())
 return dict(H_unit_face_to_image=H.tolist(),front_px=axis[0].tolist(),rear_px=axis[1].tolist(),center_px=axis[2].tolist(),vanishing_points_h=vanishing,space='image_pixels',metric_calibration=False)

def _detect_color_face(rgb):
 h,w=rgb.shape[:2];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
 dark=((hsv[:,:,2]<130)&(hsv[:,:,1]<245)).astype(np.uint8)
 dark=cv2.morphologyEx(dark,cv2.MORPH_OPEN,np.ones((15,15),np.uint8))
 n,labels,stats,centers=cv2.connectedComponentsWithStats(dark,8)
 candidates=[i for i in range(1,n) if stats[i,4]>.025*h*w and stats[i,2]>.15*w and stats[i,1]<.65*h]
 if not candidates:return None
 i=min(candidates,key=lambda i:np.linalg.norm((centers[i]-[w*.5,h*.45])/[w,h]));mask=labels==i;x,y,bw,bh,_=map(int,stats[i])
 green=((hsv[:,:,0]>65)&(hsv[:,:,0]<105)&(hsv[:,:,1]>35)&(hsv[:,:,2]>75)).astype(np.uint8)
 gn,gl,gs,gc=cv2.connectedComponentsWithStats(green,8)
 jaws=[j for j in range(1,gn) if gs[j,4]>.04*h*w and gs[j,1]+gs[j,3]>=h-3]
 end=min([int(gs[j,1]) for j in jaws],default=min(y+bh,int(h*.7)))
 if end-y<35:return None
 edges=cv2.Canny(gray,35,90);roi=np.zeros_like(edges);roi[max(0,y-10):end+3,max(0,x-8):min(w,x+bw+8)]=1;edges*=roi
 segments=cv2.HoughLinesP(edges,1,np.pi/360,threshold=25,minLineLength=max(30,int(bw*.28)),maxLineGap=12)
 if segments is None:return None
 horizontal=[]
 for a,b,c,d in segments[:,0,:]:
  length=abs(c-a)
  if length>0 and abs(d-b)/length<.35:
   mid=(b+d)/2;horizontal.append((length,mid,np.array([a,b]),np.array([c,d])))
 # Fit the outer top envelope, not an interior printed seam.
 top_points=[]
 for xx in range(x,x+bw):
  ys0=np.flatnonzero(mask[:end,xx])
  if len(ys0) and ys0[0]<=y+12:top_points.append([xx,ys0[0]])
 try:top_l,te=fit_line(top_points)
 except ValueError:return None
 tp=np.asarray(top_points)
 if np.ptp(tp[:,0])<bw*.22 or abs(top_l[1])<.9:return None
 top_y=float(np.median(tp[:,1]));bottoms=[s for s in horizontal if top_y+25<s[1]<end-5]
 groups=[]
 for seed in bottoms:
  group=[s for s in bottoms if abs(s[1]-seed[1])<5.]
  span=max(max(s[2][0],s[3][0]) for s in group)-min(min(s[2][0],s[3][0]) for s in group)
  if span>bw*.55:groups.append((sum(s[0] for s in group),group))
 if groups:
  group=max(groups,key=lambda v:v[0])[1];bottom_y=float(np.median([s[1] for s in group]))
 else:
  # Broken crease: aggregate the same brightness transition over its width.
  gy0=cv2.Sobel(gray,cv2.CV_32F,0,1,ksize=3)
  a0,b0=int(x+bw*.15),int(x+bw*.85);lo0,hi0=int(top_y)+25,end-8
  score=np.mean(gy0[:,a0:b0]<-25,axis=1);score=np.convolve(score,np.ones(5),'same')
  bottom_y=float(lo0+np.argmax(score[lo0:hi0]))
  if score[int(bottom_y)]<.6:return None
 rows=[]
 for yy in range(int(top_y)+6,int(bottom_y)-5):
  xs=np.flatnonzero(mask[yy])
  if len(xs)>5:rows.append((yy,xs[0],xs[-1]))
 if len(rows)<20:return None
 rows=np.asarray(rows)
 try:
  left,le=fit_line(rows[:,[1,0]]);right,re=fit_line(rows[:,[2,0]])
  ys,xs=np.nonzero(edges);pixels=np.c_[xs,ys]
  def transverse(seed):
   a,b=seed[2:];v=b-a;l=np.array([-v[1],v[0],v[1]*a[0]-v[0]*a[1]],float);l/=np.linalg.norm(l[:2]);good=(abs(np.c_[pixels,np.ones(len(pixels))]@l)<2.5)&(xs>=min(a[0],b[0])-4)&(xs<=max(a[0],b[0])+4)
   return fit_line(pixels[good])
  # Use the face-to-front brightness transition across the full central width.
  gy=cv2.Sobel(gray,cv2.CV_32F,0,1,ksize=3);bp=[]
  for xx in range(int(x+bw*.12),int(x+bw*.88)):
   lo=max(int(top_y)+25,int(bottom_y)-4);hi=min(end-5,int(bottom_y)+5)
   yy=lo+int(np.argmin(gy[lo:hi,xx]))
   if gy[yy,xx]<-25:bp.append([xx,yy])
  bottom_l,be=fit_line(bp)
  q=np.array([intersection(top_l,left),intersection(top_l,right),intersection(bottom_l,right),intersection(bottom_l,left)])
  if q[:,0].min()<x-25 or q[:,0].max()>x+bw+25 or q[:,1].min()<y-12 or q[:,1].max()>end: return None
  if np.linalg.norm(q[1]-q[0])<bw*.22 or np.linalg.norm(q[2]-q[3])<bw*.55:return None
  geometry=projective_geometry(q)
  if cv2.contourArea(q.astype(np.float32))<h*w*.01:return None
  return dict(quad_px=q.tolist(),edge_rms_px=[te,re,be,le],method='four_supported_lines_perspective',**geometry)
 except ValueError:return None


def detect_depth_face(rgb,xyz,base_from_color,*,insertion_tracking=None):
 """Depth selects the search area; all four output lines require real RGB edges.

 Invalid depth remains unknown. The central grip handle is excluded from the
 battery-face search and never used as a battery corner or a replacement edge.
 """
 rgb=np.asarray(rgb);xyz=np.asarray(xyz,float);T=np.asarray(base_from_color,float)
 if rgb.ndim!=3 or xyz.shape!=rgb.shape or T.shape!=(4,4) or not np.isfinite(T).all():return None
 h,w=rgb.shape[:2];yy,xx=np.indices((h,w));base=xyz@T[:3,:3].T+T[:3,3]
 valid=np.isfinite(xyz).all(2)&(xyz[:,:,2]>.065)&(xyz[:,:,2]<1.5)
 hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
 red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)).astype(np.uint8)
 # Insertion verification can reuse a measured world mouth when the rim is
 # occluded. It must never reuse the old outside-only background mask.
 roi=(xx>w*.25)&(xx<w*.83)&(yy>h*.14)&(yy<h*.44)
 handle=(xx>w*.46)&(xx<w*.66)&(yy>=h*.43)
 if insertion_tracking is None:
  n,labels,stats,_=cv2.connectedComponentsWithStats(red,8)
  candidates=[i for i in range(1,n) if stats[i,4]>300]
  if not candidates:return None
  rim=labels==max(candidates,key=lambda i:stats[i,4]);rp=base[rim&valid]
  if len(rp)<60:return None
  front_x=float(np.median(rp[:,0]))
  foreground=valid&(base[:,:,0]<front_x-.035)&(base[:,:,0]>front_x-.25)&roi&~handle
  background=valid&(base[:,:,0]>=front_x-.025)
 else:
  axis=np.asarray(insertion_tracking['axis_base'],float);mouth=np.asarray(insertion_tracking['mouth_base_m'],float)
  if axis.shape!=(3,) or mouth.shape!=(3,) or not np.isfinite(np.r_[axis,mouth]).all() or abs(np.linalg.norm(axis)-1)>1e-6:return None
  signed=(base-mouth)@axis;expected=insertion_tracking.get('expected_front_base_m')
  center=float((np.asarray(expected)-mouth)@axis) if expected is not None else 0.
  lo,hi=(center-.220,center+.035) if expected is not None else (-.300,.200)
  foreground=valid&(signed>lo)&(signed<hi)&roi&~handle&~red.astype(bool)
  background=valid&((signed<=lo)|(signed>=hi)|red.astype(bool))
  front_x=float(mouth[0])
 if foreground.sum()<100:return None
 points=np.column_stack(np.nonzero(foreground))[:,::-1].astype(np.int32)
 # Recover the 2026-11-16 attached-camera reference as a search seed only.
 # Sparse depth often misses one far corner, so its convex hull is not a face.
 # Current foreground support positions/scales the seed; RGB refits every edge.
 reference=np.array([[276.6454,80.0009],[402.8131,80.0932],[485.8121,168.8551],[209.0050,170.6607]])
 reference*=np.array([w/640.,h/480.])
 xlo,xhi=np.quantile(points[:,0],[.005,.995]);front=float(np.quantile(points[:,1],.02))-10.*h/480.
 rear=float(np.quantile(points[:,1],.98))-8.*h/480.
 if rear-front<40.*h/480.:return None
 proposal=reference.copy()
 proposal[:,0]=xlo+(reference[:,0]-reference[3,0])/(reference[2,0]-reference[3,0])*(xhi-xlo)
 proposal[:,1]=front+(reference[:,1]-reference[:2,1].mean())/(reference[2:,1].mean()-reference[:2,1].mean())*(rear-front)
 if insertion_tracking is not None and insertion_tracking.get('quad_px') is not None:
  proposal=np.asarray(insertion_tracking['quad_px'],float)
  if proposal.shape!=(4,2) or not np.isfinite(proposal).all():return None
 if proposal[0,1]>h*.34 or proposal[2:,1].mean()>h*.46:return None
 gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
 processing=gray.copy();processing[~roi|handle]=int(np.median(gray[roi&~handle]))
 enhanced=cv2.createCLAHE(clipLimit=3.,tileGridSize=(8,8)).apply(processing)
 edge=cv2.Canny(enhanced,20,55)>0
 edge&=roi&~handle&~background
 ey,ex=np.nonzero(edge);pixels=np.column_stack((ex,ey)).astype(float)
 gx=cv2.Sobel(enhanced,cv2.CV_32F,1,0,ksize=3);gy=cv2.Sobel(enhanced,cv2.CV_32F,0,1,ksize=3)
 rng=np.random.default_rng(24);lines=[];errors=[];supports=[]
 for index,(a,b) in enumerate(zip(proposal,np.roll(proposal,-1,axis=0))):
  direction=b-a;length=float(np.linalg.norm(direction))
  if length<35:return None
  direction/=length;normal=np.array([-direction[1],direction[0]])
  relative=pixels-a;along=relative@direction;distance=relative@normal
  gradient=np.column_stack((gx[ey,ex],gy[ey,ex]));gn=np.linalg.norm(gradient,axis=1)
  keep=(along>-.08*length)&(along<1.08*length)&(abs(distance)<(32. if index==0 else 14.))&(abs(gradient@normal)>.65*np.maximum(gn,1))
  if index==0:
   front_y=float(np.mean(proposal[:2,1])) if insertion_tracking is not None and insertion_tracking.get('quad_px') is not None else float(np.quantile(points[:,1],.02));keep&=(pixels[:,1]>front_y-12)&(pixels[:,1]<front_y+14)
  if index==2:keep&=gy[ey,ex]<0
  selected=pixels[keep]
  if len(selected)<20:return None
  best=None
  for _ in range(180):
   pair=selected[rng.choice(len(selected),2,replace=False)];v=pair[1]-pair[0];norm=np.linalg.norm(v)
   if norm<15:continue
   v/=norm
   if abs(v@direction)<np.cos(np.radians(35 if index==0 else 14)):continue
   if index in (0,2) and abs(v[1])>np.sin(np.radians(8)):continue
   l=np.array([-v[1],v[0],v[1]*pair[0,0]-v[0]*pair[0,1]])
   residual=abs(np.c_[selected,np.ones(len(selected))]@l);inliers=residual<1.6
   if inliers.sum()<18 or np.ptp(selected[inliers]@direction)<length*.55:continue
   score=float(inliers.sum())-.3*float(np.median(abs((selected[inliers]-a)@normal)))
   if best is None or score>best[0]:best=(score,inliers)
  if best is None:return None
  support=selected[best[1]]
  try:l,error=fit_line(support)
  except ValueError:return None
  if error>1.6:return None
  lines.append(l);errors.append(error);supports.append(int(len(support)))
 try:q=np.asarray([intersection(lines[3],lines[0]),intersection(lines[0],lines[1]),intersection(lines[1],lines[2]),intersection(lines[2],lines[3])]);geometry=projective_geometry(q)
 except ValueError:return None
 if np.max(np.linalg.norm(q-proposal,axis=1))>27:return None
 if np.linalg.norm(q[1]-q[0])<w*.10 or np.linalg.norm(q[2]-q[3])<w*.25:return None
 if not .25<np.linalg.norm(q[1]-q[0])/np.linalg.norm(q[2]-q[3])<.90:return None
 if q[:,1].max()>=h*.44 or cv2.contourArea(q.astype(np.float32))<w*h*.015:return None
 inside=np.zeros((h,w),np.uint8);cv2.fillConvexPoly(inside,np.rint(q).astype(np.int32),1)
 observed=(inside>0)&valid
 if observed.sum()<80 or np.mean(background[observed])>.08:return None
 return dict(quad_px=q.tolist(),edge_rms_px=errors,edge_support_pixels=supports,
  method='depth_background_removed_four_rgb_lines' if insertion_tracking is None else 'insertion_depth_supported_four_rgb_lines',depth_support_pixels=int(observed.sum()),
  measured_background_removed_pixels=int((background&roi).sum()),handle_excluded=True,
  invalid_depth_treated_as='unknown',cabinet_front_x_m=front_x,
  search_reference='20261116_perspective_seed_refitted_on_current_rgb',**geometry)

def detect_face(rgb,*,xyz=None,base_from_color=None,insertion_tracking=None):
 if xyz is not None and base_from_color is not None:
  if insertion_tracking is not None and insertion_tracking.get('quad_px') is None:
   outside=detect_depth_face(rgb,xyz,base_from_color)
   if outside is not None:return outside
   color=_detect_color_face(rgb)
   if color is not None:insertion_tracking=dict(insertion_tracking,quad_px=color['quad_px'])
  return detect_depth_face(rgb,xyz,base_from_color,insertion_tracking=insertion_tracking)
 return _detect_color_face(rgb)

def select_depth_consensus(observations,*,minimum=3,tolerance_px=9.):
 """Select one actually observed frame; do not draw averaged invented corners."""
 available=[(i,r) for i,r in enumerate(observations) if r is not None]
 if len(available)<minimum:return None
 quads=np.asarray([r['quad_px'] for _,r in available],float)
 median=np.median(quads,axis=0);deviations=np.max(np.linalg.norm(quads-median,axis=2),axis=1)
 accepted=np.flatnonzero(deviations<=tolerance_px)
 if len(accepted)<minimum:return None
 distances=np.linalg.norm(quads[accepted]-median,axis=2).mean(1)
 chosen=int(accepted[np.argmin(distances)]);index,result=available[chosen]
 result=dict(result,consensus_frames=int(len(accepted)),sampled_frames=len(observations),
  consensus_max_corner_deviation_px=float(deviations[accepted].max()),
  consensus_corner_std_px=np.std(quads[accepted],axis=0).tolist(),
  tracking='observed_frame_consensus',selected_sample_index=index)
 return index,result

def select_depth_preview(observations):
 """Single supported exposure is usable; consensus is advisory for preview."""
 result=select_depth_consensus(observations)
 if result:
  index,value=result;return index,dict(value,consensus_required=False)
 available=[(i,r) for i,r in enumerate(observations) if r is not None]
 if not available:return None
 index,value=min(available,key=lambda pair:float(np.mean(pair[1]['edge_rms_px'])))
 return index,dict(value,consensus_frames=1,sampled_frames=len(observations),
  tracking='single_valid_frame',consensus_required=False,selected_sample_index=index)

class PerspectiveTracker:
 def __init__(self):
  self.history=deque(maxlen=5);self.pending=deque(maxlen=3);self.q=None;self.last_good=None;self.last_stamp=None;self.last_seq=None;self.last_result=None
 def update(self,observation,stamp,sequence):
  if sequence==self.last_seq:return self.last_result
  if self.last_stamp is not None and stamp<=self.last_stamp:return None
  if self.last_stamp is not None and stamp-self.last_stamp>1.:
   self.history.clear();self.pending.clear();self.q=None
  self.last_stamp=stamp;self.last_seq=sequence;accepted=False
  if observation is not None:
   q=np.array(observation['quad_px'],float)
   try:projective_geometry(q)
   except ValueError:observation=None
   else:
    if self.q is not None and np.max(np.linalg.norm(q-self.q,axis=1))>10:
     if self.pending and np.max(np.linalg.norm(q-self.pending[-1],axis=1))>4:self.pending.clear()
     self.pending.append(q)
     if len(self.pending)==3:
      self.history.clear();self.q=None;self.pending.clear();accepted=True
    else:self.pending.clear();accepted=True
    if accepted:
     self.history.append(q);median=np.median(self.history,axis=0);self.q=median if self.q is None else .35*median+.65*self.q;self.last_good=stamp
  if self.q is None or self.last_good is None or stamp-self.last_good>.6:
   self.last_result=None;return None
  r=dict(quad_px=self.q.tolist(),tracking=('tracked' if len(self.history)>=3 else 'acquiring') if accepted else 'held_recent',samples=len(self.history),age_since_observed_s=stamp-self.last_good,**projective_geometry(self.q))
  if accepted:r['edge_rms_px']=observation['edge_rms_px']
  self.last_result=r;return r

