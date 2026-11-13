"""Static-only scan alignment and repeatable head obstacle observations."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def thin(points, pitch, cap=4000):
    if not len(points):
        return points
    _, idx = np.unique(np.floor(points / pitch).astype(np.int32), axis=0, return_index=True)
    pts = points[idx]
    return pts[::max(1, int(np.ceil(len(pts) / cap)))]


def align(bottom, head):
    """Bounded point-to-plane correction to the CAD pose, validated on held-out points."""
    b = thin(bottom, .085)
    h = thin(head, .10, 2400)
    if min(len(b),len(h)) < 100:
        raise ValueError('sparse_scene')
    tree = cKDTree(b)
    ni=tree.query(b,k=8)[1]
    local=b[ni]-b[ni].mean(axis=1)[:,None,:]
    values,vectors=np.linalg.eigh(local.transpose(0,2,1)@local)
    normals=vectors[:,:,0]
    planar=(values[:,0]<.22*values[:,1]) & (values[:,1]>.004)
    train=h[1::2]; test=h[::2]
    rot=np.eye(3); trans=np.zeros(3)
    for _ in range(4):
        p=train@rot.T+trans
        d,i=tree.query(p)
        use=(d<.22)&planar[i]
        if use.sum()<45:raise ValueError('insufficient_overlap')
        n=normals[i[use]]; q=p[use]
        a=np.column_stack((n,np.cross(q,n)))
        residual=np.einsum('ij,ij->i',n,b[i[use]]-q)
        w=np.sqrt(np.minimum(1.,.04/(abs(residual)+1e-8)))
        aw=a*w[:,None]
        sv=np.linalg.svd(aw,compute_uv=False)
        if sv[-1]/sv[0] < .022:raise ValueError('degenerate_geometry')
        step=np.linalg.lstsq(aw,residual*w,rcond=None)[0]
        dr=Rotation.from_rotvec(step[3:]).as_matrix()
        rot=dr@rot; trans=dr@trans+step[:3]
        if np.linalg.norm(trans)>.10 or Rotation.from_matrix(rot).magnitude()>np.deg2rad(4):
            raise ValueError('outside_CAD_prior')
    p=test@rot.T+trans
    d,i=tree.query(p);use=(d<.20)&planar[i]
    if use.sum()<40 or use.mean()<.12:raise ValueError('held_out_overlap')
    err=abs(np.einsum('ij,ij->i',normals[i[use]],p[use]-b[i[use]]))
    if np.median(err)>.035 or np.percentile(err,90)>.09:raise ValueError('held_out_residual')
    return rot,trans,dict(overlap=int(use.sum()),median=float(np.median(err)),p90=float(np.percentile(err,90)))


def confirmed(points, previous):
    if len(previous)<2 or not len(points) or any(not len(p) for p in previous):return points[:0]
    keep=np.ones(len(points),bool)
    for p in previous:
        keep &= cKDTree(p).query(points,distance_upper_bound=.085)[0]<.085
    return points[keep]
