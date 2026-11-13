import numpy as np
from scene_registration import transform

def compensate(points, acquisition_pose, reference_pose):
 return transform(points,np.linalg.inv(reference_pose)@acquisition_pose)

def persistent_points(frames,min_frames=3,min_range=1.2,size=.12):
 from scipy.spatial import cKDTree
 from scene_registration import voxel
 cleaned=[]
 for p in frames:
  p=np.asarray(p);p=p[np.isfinite(p).all(axis=1)&(np.linalg.norm(p,axis=1)>min_range)]
  if len(p):cleaned.append(p)
 if not cleaned:return np.empty((0,3))
 candidates=voxel(np.vstack(cleaned),size)
 votes=np.zeros(len(candidates),dtype=int)
 for p in cleaned:
  distances,_=cKDTree(p).query(candidates,workers=1)
  votes+=distances<.15
 return candidates[votes>=min_frames]

def spatial_split(points,coordinates=None,fold=0):
 coords=points if coordinates is None else coordinates;k=np.floor(coords/.6).astype(np.int64);val=((k[:,0]*73856093+k[:,1]*19349663+k[:,2]*83492791)%5)==fold
 return points[~val],points[val]
