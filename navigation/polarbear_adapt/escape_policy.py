"""Bounded Local-only withdrawal and optional, explicitly permitted sparse exceptions."""
import math
import numpy as np
from matplotlib.path import Path
from scipy.ndimage import label
from scipy.spatial import cKDTree
MAX_CELLS=6
MAX_CLUSTER=3
MIN_SEPARATION=.10

def key(xy,res):return tuple(np.rint(np.asarray(xy)/(res/4)).astype(np.int64))

def contacts(grid,pose):
 v=grid._vertices(pose);h=grid.res/2;lo=np.floor((v.min(0)-grid.origin)/grid.res).astype(int)-1;hi=np.floor((v.max(0)-grid.origin)/grid.res).astype(int)+1
 j,i=np.mgrid[lo[1]:hi[1]+1,lo[0]:hi[0]+1];i,j=i.ravel(),j.ravel();inside=(i>=0)&(j>=0)&(i<grid.grid.shape[1])&(j<grid.grid.shape[0]);values=np.full(len(i),-1);values[inside]=grid.grid[j[inside],i[inside]];use=(values==100)|(values<0)
 if not use.any():return {},set()
 xy=grid.origin+(np.c_[i[use],j[use]]+.5)*grid.res;values=values[use];within=Path(v).contains_points(xy);touch=within.copy();distance=np.full(len(xy),np.inf)
 for a,b in zip(v,np.roll(v,-1,axis=0)):
  delta=b-a;ll=float(delta@delta);t=np.clip(((xy-a)@delta)/ll,0,1);distance=np.minimum(distance,np.linalg.norm(xy-(a+t[:,None]*delta),axis=1));low=np.zeros(len(xy));high=np.ones(len(xy))
  for axis in (0,1):
   offset=xy[:,axis]-a[axis]
   if abs(delta[axis])<1e-12:low[abs(offset)>h]=2
   else:u,w=(offset-h)/delta[axis],(offset+h)/delta[axis];low=np.maximum(low,np.minimum(u,w));high=np.minimum(high,np.maximum(u,w))
  touch|=low<=high
 depth=np.maximum(0.,grid.res/math.sqrt(2)+np.where(within,distance,-distance))
 orientation=np.sign(np.cross(v,np.roll(v,-1,axis=0)).sum());faces=[]
 for a,b in zip(v,np.roll(v,-1,axis=0)):
  e=b-a;faces.append(orientation*(e[0]*(xy[:,1]-a[1])-e[1]*(xy[:,0]-a[0]))/np.linalg.norm(e))
 faces=np.asarray(faces).T
 occupied={key(p,grid.res):dict(xy=p,depth=float(d),faces=f,active=np.flatnonzero(f<=max(grid.res*math.sqrt(2),float(f.min())+1e-8))) for p,d,f,z,t in zip(xy,depth,faces,values,touch) if t and z==100};unknown={key(p,grid.res) for p,z,t in zip(xy,values,touch) if t and z<0};return occupied,unknown

def sparse_cells(grid):
 hard=grid.grid==100;components,count=label(hard,np.ones((3,3)));ys,xs=np.where(hard)
 if not len(xs):return {}
 xy=grid.origin+(np.c_[xs,ys]+.5)*grid.res;ids=components[ys,xs];tree=cKDTree(xy);sizes=np.bincount(ids,minlength=count+1);result={}
 for cid in np.flatnonzero((sizes>0)&(sizes<=MAX_CLUSTER)):
  if cid==0 or sizes[cid]*grid.res**2>.0012+1e-9:continue
  rows=np.flatnonzero(ids==cid);near=set(k for group in tree.query_ball_point(xy[rows],MIN_SEPARATION+math.sqrt(2)*grid.res+1e-8) for k in group if ids[k]!=cid)
  if near:
   delta=np.abs(xy[rows,None,:]-xy[np.asarray(sorted(near))][None,:,:]);gap=np.linalg.norm(np.maximum(delta-grid.res,0.),axis=2)
   if gap.min()<MIN_SEPARATION-1e-8:continue
  group=frozenset(key(p,grid.res) for p in xy[rows]);centres={key(p,grid.res):p for p in xy[rows]}
  for k in group:result[k]=(group,centres)
 return result

def admissible(grid,baseline,path,ignored=(),tolerance=.003):
 ignored=set(ignored);previous={k:v['faces'] for k,v in baseline.items() if k not in ignored};initial={k:baseline[k] for k in previous}
 for pose in path:
  now,unknown=contacts(grid,pose)
  if unknown:return False,'unknown'
  now={k:v for k,v in now.items() if k not in ignored}
  if set(now)-set(initial):return False,'new_obstacle'
  for k,v in now.items():
   active=initial[k]['active'];prior=previous.get(k)
   if prior is None:return False,'overlap_reentry'
   if np.any(v['faces'][active]>np.minimum(initial[k]['faces'][active],prior[active])+tolerance):return False,'deeper_overlap'
  previous={k:now[k]['faces'] if k in now else None for k in initial}
 return True,'ok'

def choose(grid,start,target,allow_sparse=True,reserve=.52):
 start=np.asarray(start,float);target=np.asarray(target,float)
 if not np.isfinite(start).all() or not np.isfinite(target).all():return None
 baseline,unknown=contacts(grid,start)
 if unknown:return None
 wanted=target[:2]-start[:2];wanted=wanted/max(np.linalg.norm(wanted),1e-9);step=min(.01,grid.res/2);distances=np.linspace(0,reserve,int(math.ceil(reserve/step))+1)
 def candidates(sparse):
  eligible=sparse_cells(grid) if sparse else {};out=[]
  for angle in start[2]+np.arange(16)*math.pi/8:
   direction=np.array([math.cos(angle),math.sin(angle)]);full=np.tile(start,(len(distances),1));full[:,:2]+=distances[:,None]*direction;ignored=set();groups=set();centres={};clear_distance=None
   # Stop at the first actual withdrawal point; reserve the unchanged native
   # .32m/s * .9s sweep from there, plus 2cm. Do not require travel past it.
   for distance,pose in zip(distances,full):
    if distance>.18+1e-9:break
    hits,unk=contacts(grid,pose)
    if not unk and not(set(hits)-set(eligible)):
     clear_distance=float(distance);break
   if clear_distance is None:continue
   check_distance=min(reserve,clear_distance+.32*.9+.02)
   path=full[distances<=check_distance+step]
   if sparse:
    for pose in path:
     hits,unk=contacts(grid,pose)
     if unk:break
     for k in hits:
      if k in eligible:
       group,points=eligible[k];groups.add(group);ignored.update(group);centres.update(points)
    if len(groups)>2 or len(ignored)>MAX_CELLS or not ignored:continue
   ok,_=admissible(grid,baseline,path,ignored)
   if not ok:continue
   out.append(dict(mode='sparse' if sparse else 'withdraw',direction=direction,clear_distance=clear_distance,ignored=[centres[k].tolist() for k in sorted(ignored)],ignored_keys=ignored,baseline=baseline,goal_alignment=float(direction@wanted),path=path))
  return out
 normal=candidates(False)
 if normal:return max(normal,key=lambda p:(p['goal_alignment'],-p['clear_distance']))
 if allow_sparse:
  special=candidates(True)
  if special:return max(special,key=lambda p:(p['goal_alignment'],-len(p['ignored']),-p['clear_distance']))
 return None
