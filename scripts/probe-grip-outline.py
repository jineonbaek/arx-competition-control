"""Read-only experiment: robust fixed-reference contour registration."""
from pathlib import Path
import sys,time,json,cv2,numpy as np
from scipy.optimize import least_squares
from scipy.ndimage import map_coordinates
root=Path('/home/arx/arx-place-hold-20261116');sys.path.insert(0,str(root))
from agent.docking_grip import GripTracker
def boundary(shape):
 mask=shape['mask'];h,w=mask.shape
 edge=np.zeros_like(mask);contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE);cv2.drawContours(edge,contours,-1,255,1)
 yy,xx=np.nonzero(edge);good=(yy>h*.12+3)&(yy<h*.88-3)&(xx>w*.18+3)&(xx<w*.82-3)
 pts=np.c_[xx[good],yy[good]].astype(float)[::4]
 clean=np.zeros_like(mask);clean[yy[good],xx[good]]=255
 distance=cv2.distanceTransform(255-clean,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
 return pts,distance
def fit(a,b):
 ref=GripTracker._shape(a);cur=GripTracker._shape(b);p,dist=boundary(ref);q,other=boundary(cur);center=ref['center']
 def transform(points,x,inverse=False):
  dx,dy,angle,scale=x;angle/=100;scale=np.exp(scale/100)
  c,s=np.cos(angle),np.sin(angle);R=np.array([[c,-s],[s,c]])
  return ((points-center-[dx,dy])@R/scale+center) if inverse else ((points-center)@R.T*scale+center+[dx,dy])
 def residual(x):
  forward=transform(p,x);back=transform(q,x,True)
  return np.r_[map_coordinates(other,forward[:,::-1].T,order=1,mode='constant',cval=50),map_coordinates(dist,back[:,::-1].T,order=1,mode='constant',cval=50)]
 start=time.monotonic();r=least_squares(residual,[0.,0.,0.,0.],bounds=([-30,-30,-26,-20],[30,30,26,20]),loss='soft_l1',f_scale=2.,diff_step=.01,max_nfev=100)
 e=residual(r.x);return dict(x=r.x.tolist(),median=float(np.median(e)),p90=float(np.quantile(e,.9)),coverage=float(np.mean(e<2)),seconds=time.monotonic()-start)
if __name__=='__main__':
 p=root/'reports/d405-roll-diagnostics';a=cv2.imread(str(p/'reference.jpg'))[:,:,::-1];rows=[]
 for i in range(15):
  b=cv2.imread(str(p/f'frame-{i}.jpg'))[:,:,::-1];r=fit(a,b);rows.append(r);print(i,r,flush=True)
 (p/'outline-probe.json').write_text(json.dumps(rows,indent=2))
