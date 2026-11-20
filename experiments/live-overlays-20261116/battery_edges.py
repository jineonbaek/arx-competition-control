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

def detect_face(rgb):
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
