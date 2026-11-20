"""Observed red-frame straight edges; hidden edges are not invented."""
from collections import deque
import numpy as np,cv2
from d405_battery_edges import fit_line,intersection,projective_geometry
NAMES=('top','right','bottom','left')
def line_angle_error(battery,reference):
 b=np.asarray(battery,float);r=np.asarray(reference,float)
 if b.shape!=(2,2) or r.shape!=(2,2) or not np.isfinite(np.r_[b.ravel(),r.ravel()]).all():raise ValueError('Invalid edge')
 a=b[1]-b[0];c=r[1]-r[0]
 if min(np.linalg.norm(a),np.linalg.norm(c))<5:raise ValueError('Edge too short')
 return float((np.degrees(np.arctan2(a[1],a[0])-np.arctan2(c[1],c[0]))+90)%180-90)

def detect_rim(rgb):
 h,w=rgb.shape[:2];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
 red=(((hsv[:,:,0]<12)|(hsv[:,:,0]>168))&(hsv[:,:,1]>90)&(hsv[:,:,2]>60)).astype(np.uint8)
 n,l,st,c=cv2.connectedComponentsWithStats(red,8);ids=[i for i in range(1,n) if st[i,4]>500 and st[i,2]>60 and st[i,3]>25]
 if not ids:return None
 i=max(ids,key=lambda i:st[i,4]);x,y,bw,bh,_=map(int,st[i]);mask=l==i;points={k:[] for k in NAMES}
 for xx in range(x+int(.12*bw),x+int(.88*bw)):
  ys=np.flatnonzero(mask[:,xx]);runs=np.split(ys,np.flatnonzero(np.diff(ys)>1)+1) if len(ys) else [];runs=[v for v in runs if len(v)>=3]
  if not runs:continue
  points['top'].append([xx,runs[0][0]])
  if len(runs)>=2 and runs[-1][0]-runs[0][-1]>.25*bh:points['bottom'].append([xx,runs[-1][-1]])
 for yy in range(y+int(.1*bh),y+int(.9*bh)):
  xs=np.flatnonzero(mask[yy])
  if len(xs)==0:continue
  if xs[0]<x+.22*bw:points['left'].append([xs[0],yy])
  if xs[-1]>x+.78*bw:points['right'].append([xs[-1],yy])
 edges={}
 for name,p in points.items():
  if len(p)<12:continue
  p=np.asarray(p,float);axis=0 if name in ('top','bottom') else 1
  if np.ptp(p[:,axis])<(bw*.35 if axis==0 else max(12,bh*.2)):continue
  try:line,rms=fit_line(p)
  except ValueError:continue
  if rms>2.5:continue
  if axis==0:
   if abs(line[1])<.85:continue
   a,b=np.percentile(p[:,0],[0,100]);segment=np.array([[a,-(line[0]*a+line[2])/line[1]],[b,-(line[0]*b+line[2])/line[1]]])
  else:
   if abs(line[0])<.85:continue
   a,b=np.percentile(p[:,1],[0,100]);segment=np.array([[-(line[1]*a+line[2])/line[0],a],[-(line[1]*b+line[2])/line[0],b]])
  edges[name]=dict(line_h=line.tolist(),segment_px=segment.tolist(),rms_px=rms,support=len(p))
 if 'top' not in edges:return None
 q=None;geometry={}
 if len(edges)==4:
  try:
   ls={k:np.array(v['line_h']) for k,v in edges.items()};q=np.array([intersection(ls['top'],ls['left']),intersection(ls['top'],ls['right']),intersection(ls['bottom'],ls['right']),intersection(ls['bottom'],ls['left'])]);geometry=projective_geometry(q);q=q.tolist()
  except ValueError:q=None
 return dict(edges=edges,quad_px=q,hidden_edges=[k for k in NAMES if k not in edges],method='robust_four_line_perspective',metric_calibration=False,**{k:v for k,v in geometry.items() if k!='metric_calibration'})

class RimTracker:
 def __init__(self):self.hist={};self.last_good={};self.seq=None;self.previous=None
 def update(self,obs,stamp,seq):
  if seq==self.seq:return self.previous
  self.seq=seq;result={}
  for name in NAMES:
   edge=(obs or {}).get('edges',{}).get(name)
   if edge is not None:
    points=np.array(edge['segment_px']);hist=self.hist.setdefault(name,deque(maxlen=5))
    if hist and np.max(np.linalg.norm(points-hist[-1],axis=1))>15:hist.clear()
    hist.append(points);self.last_good[name]=(stamp,edge)
   if name not in self.last_good or stamp-self.last_good[name][0]>.6:continue
   age=stamp-self.last_good[name][0];points=np.median(self.hist[name],axis=0);line=np.cross(np.r_[points[0],1],np.r_[points[1],1]);line/=np.linalg.norm(line[:2]);result[name]=dict(self.last_good[name][1],segment_px=points.tolist(),line_h=line.tolist(),held_recent=edge is None,age_s=age)
  if 'top' not in result:self.previous=None;return None
  q=None;geometry={}
  if len(result)==4:
   try:
    ls={k:np.array(v['line_h']) for k,v in result.items()};q=np.array([intersection(ls['top'],ls['left']),intersection(ls['top'],ls['right']),intersection(ls['bottom'],ls['right']),intersection(ls['bottom'],ls['left'])]);geometry=projective_geometry(q);q=q.tolist()
   except ValueError:q=None
  self.previous=dict(edges=result,quad_px=q,hidden_edges=[k for k in NAMES if k not in result],metric_calibration=False,**{k:v for k,v in geometry.items() if k!='metric_calibration'});return self.previous

