"""Generate a conservative interior of the fixed base_link below the LIFT deck."""
import hashlib
import sys
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_erosion

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'dual_flex'))
from mesh_filter import urdf_geometry

PITCH=.0025
DECK=.183


def parity(tri,axes,shape,lower):
    # Sample along one principal axis. Ambiguous edge hits and open rays
    # cannot certify an interior. All three axes must agree.
    a,b,c=axes
    events=np.zeros(tuple(shape[[a,b,c]]),np.int32)
    count=np.zeros(events.shape[:2],np.int32)
    ambiguous=np.zeros_like(count,bool)
    for start in range(0,len(tri),256):
        t=tri[start:start+256][:,:,axes]/PITCH-lower[list(axes)]
        u=t[:,1]-t[:,0];v=t[:,2]-t[:,0]
        det=u[:,0]*v[:,1]-u[:,1]*v[:,0]
        good=np.abs(det)>1e-10;t,u,v,det=t[good],u[good],v[good],det[good]
        lo=np.maximum(0,np.ceil(t[:,:,:2].min(axis=1)).astype(int))
        hi=np.minimum(shape[[a,b]]-1,np.floor(t[:,:,:2].max(axis=1)).astype(int))
        width=np.maximum(hi-lo+1,0);n=width.prod(axis=1)
        ids=np.repeat(np.arange(len(t)),n)
        if not len(ids):continue
        k=np.arange(len(ids))-np.repeat(np.cumsum(n)-n,n)
        x=lo[ids,0]+k//width[ids,1];y=lo[ids,1]+k%width[ids,1]
        dx=x-t[ids,0,0];dy=y-t[ids,0,1]
        s=(dx*v[ids,1]-dy*v[ids,0])/det[ids]
        r=(u[ids,0]*dy-u[ids,1]*dx)/det[ids]
        edge=np.minimum.reduce([s,r,1-s-r]); ambiguous[x[abs(edge)<1e-9],y[abs(edge)<1e-9]]=True
        hit=edge>1e-9
        z=t[ids,0,2]+s*u[ids,2]+r*v[ids,2]
        zi=np.floor(z[hit]).astype(int)+1
        np.add.at(count,(x[hit],y[hit]),1)
        inside=zi<shape[c]
        np.add.at(events,(x[hit][inside],y[hit][inside],np.maximum(0,zi[inside])),1)
    occupied=(np.cumsum(events,axis=2)%2==1)&((count%2==0)&~ambiguous)[:,:,None]
    return np.transpose(occupied,np.argsort(axes))


def build(urdf,out):
    selected=[p for p in urdf_geometry(urdf) if p['link']=='base_link']
    if len(selected)!=1:raise ValueError('Expected one fixed base_link STL')
    part=selected[0];tri=part['triangles']
    lower=np.floor(tri.min(axis=(0,1))/PITCH).astype(int)-2
    upper=np.ceil(tri.max(axis=(0,1))/PITCH).astype(int)+2
    upper[2]=int(np.floor(DECK/PITCH))
    shape=upper-lower+1
    solid=np.ones(tuple(shape),bool)
    for axes in [(0,1,2),(1,2,0),(2,0,1)]:
        solid &= parity(tri,axes,shape,lower)
    # Drop the surface band, including the clipping plane at the deck.
    solid=binary_erosion(solid,structure=np.ones((3,3,3),bool),iterations=1)
    if not solid.any():raise ValueError('No certified base interior')
    np.savez_compressed(out,solid=solid,lower=lower,pitch=PITCH,deck=DECK,
                        link='base_link',stl_sha256=part['sha256'],
                        urdf_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest())
    print('certified interior cells:',int(solid.sum()),'pitch:',PITCH)


if __name__=='__main__':build(*sys.argv[1:])
