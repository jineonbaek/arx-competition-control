"""Offline robust point-to-plane registration with degeneracy diagnostics."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

def transform(p,T):return p@T[:3,:3].T+T[:3,3]
def voxel(p,size=.10):
 p=np.asarray(p);p=p[np.isfinite(p).all(axis=1)];_,i=np.unique(np.floor(p/size).astype(np.int64),axis=0,return_index=True);return p[i]
def prepare_target(p):
 p=np.asarray(p);tree=cKDTree(p);dist,idx=tree.query(p,k=min(20,len(p)),workers=1);near=p[idx];center=near.mean(axis=1);delta=near-center[:,None,:];cov=np.einsum('nki,nkj->nij',delta,delta)/near.shape[1];e,v=np.linalg.eigh(cov);valid=(e[:,0]/np.maximum(e.sum(axis=1),1e-12)<.08)&(dist[:,-1]<.8)
 return (p,tree,v[:,:,0],valid)
def register(source,target,initial,max_iter=35):
 p,tree,normals,valid=target;T=initial.copy();rank=0;condition=float('inf');count=0
 for iteration in range(max_iter):
  x=transform(source,T);dist,ids=tree.query(x,workers=1);mask=(dist<.30)&valid[ids];x=x[mask];q=p[ids[mask]];n=normals[ids[mask]]
  if len(x)<40:break
  residual=np.sum((x-q)*n,axis=1);keep=np.abs(residual)<min(.15,max(.025,np.quantile(np.abs(residual),.85)));x=x[keep];n=n[keep];residual=residual[keep];count=len(x)
  A=np.c_[np.cross(x,n),n];weight=np.minimum(1.,.025/np.maximum(np.abs(residual),1e-12));Aw=A*np.sqrt(weight)[:,None];bw=-residual*np.sqrt(weight)
  singular=np.linalg.svd(Aw,compute_uv=False);rank=int(np.sum(singular>singular[0]*1e-3));condition=float(singular[0]/max(singular[-1],1e-15));step=np.linalg.lstsq(Aw,bw,rcond=1e-3)[0]
  if np.linalg.norm(step[:3])>.15 or np.linalg.norm(step[3:])>.25:break
  R=Rotation.from_rotvec(step[:3]).as_matrix();T[:3,:3]=R@T[:3,:3];T[:3,3]=R@T[:3,3]+step[3:]
  if np.linalg.norm(step)<1e-6:break
 x=transform(source,T);dist,ids=tree.query(x,workers=1);mask=(dist<.20)&valid[ids];res=np.abs(np.sum((x[mask]-p[ids[mask]])*normals[ids[mask]],axis=1));coverage=float(mask.mean());rms=float(np.sqrt(np.mean(np.minimum(res,.1)**2))) if len(res) else 1.
 return {'T':T,'rank':rank,'condition':condition,'correspondences':count,'train_coverage':coverage,'train_rms_m':rms,'score':rms+.05*(1-coverage),'iterations':iteration+1}
def heldout(source,target,T):
 x=transform(source,T);tree=cKDTree(target);d,_=tree.query(x,workers=1);back,_=cKDTree(x).query(target,workers=1)
 return {'source_within_5cm':float(np.mean(d<.05)),'source_within_10cm':float(np.mean(d<.10)),'target_within_10cm':float(np.mean(back<.10)),'overlap_rmse_under_20cm_m':float(np.sqrt(np.mean(d[d<.2]**2))) if np.any(d<.2) else None}
