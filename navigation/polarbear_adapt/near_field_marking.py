"""Near-field persistent mark promotion; never substitutes for Local or free rays."""
from collections import deque
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

class NearFieldPromotion:
 def __init__(self,near_m=1.,support_m=.05,window_s=.35):
  if not all(math.isfinite(x) and x>0 for x in (near_m,support_m,window_s)):raise ValueError('positive finite promotion bounds required')
  self.near_m=near_m;self.support_m=support_m;self.window_s=window_s;self.history=deque();self.last=-math.inf
 def keep(self,points,origin,stamp):
  p=np.asarray(points);o=np.asarray(origin)
  if p.ndim!=2 or p.shape[1]!=3 or o.shape!=(3,):raise ValueError('common-frame Nx3 points and 3D origin required')
  keep=np.zeros(len(p),bool)
  if not math.isfinite(stamp) or stamp<=self.last or not np.isfinite(o).all():return keep
  self.last=stamp
  while self.history and stamp-self.history[0][0]>self.window_s+1e-9:self.history.popleft()
  finite=np.isfinite(p).all(axis=1);near=finite&(np.linalg.norm(p[:,:2]-o[:2],axis=1)<=self.near_m)
  keep[finite&~near]=True;tree=None
  if near.any():
   unique,inverse=np.unique(p[near],axis=0,return_inverse=True);tree=cKDTree(unique)
   spatial=tree.query_ball_point(unique,self.support_m,return_length=True)>=3
   prior=np.zeros(len(unique),int)
   for _,old in self.history:
    if old is not None:prior+=old.query(unique,distance_upper_bound=self.support_m)[0]<self.support_m
   keep[near]=(spatial|(prior>=2))[inverse]
  self.history.append((stamp,tree));return keep

def model_orientation(body_xyzw,rear=False):
 """Keep actual up axis; retain vehicle-relative rear horizontal mask heading."""
 body=Rotation.from_quat(body_xyzw)
 return (body*Rotation.from_rotvec([0.,0.,math.pi if rear else 0.])).as_quat()
