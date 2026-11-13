"""Positive visibility through an old cell from a parked, elevated scanner."""
import numpy as np


def evidence(grid, xy, origin, endpoints, sensor, base_z, res, veto_neighbours=True):
    """Return reference-cell indices crossed low by real rays, plus current hit vetoes."""
    empty=np.empty(0,dtype=int)
    if not len(endpoints) or not len(xy):return empty,empty
    rays=endpoints-sensor
    ranges=np.linalg.norm(rays[:,:2],axis=1)
    az=np.floor((np.arctan2(rays[:,1],rays[:,0])+np.pi)/(.5*np.pi/180)).astype(int)%720
    height=endpoints[:,2]-base_z
    hits=(height>.055)&(height<=1.35)
    cells=np.floor((endpoints[hits,:2]-origin)/res).astype(int)
    veto=np.zeros(grid.shape,bool)
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            x=cells[:,0]+dx;y=cells[:,1]+dy
            inside=(x>=0)&(y>=0)&(x<grid.shape[1])&(y<grid.shape[0])
            veto[y[inside],x[inside]]=True
    ref=np.floor((xy-origin)/res).astype(int)
    veto_idx=np.flatnonzero(veto[ref[:,1],ref[:,0]])
    nearest=np.full(720,np.inf);np.minimum.at(nearest,az[hits],ranges[hits])
    nearest=np.minimum(nearest,np.minimum(np.roll(nearest,1),np.roll(nearest,-1)))
    down=(height>=-.06)&(height<=.055)&(rays[:,2]<-.1)&(ranges>.6)
    # Sample the part of each real ray that passes 6–18 cm above the floor.
    # Require 12 cm of measured range beyond every crossed cell.
    idx=np.flatnonzero(down)
    if not len(idx):return empty,veto_idx
    rr=ranges[idx];direction=rays[idx,:2]/rr[:,None]
    begin=(base_z+.18-sensor[2])/rays[idx,2]*rr
    end=np.minimum((base_z+.06-sensor[2])/rays[idx,2]*rr,
                   np.minimum(rr,nearest[az[idx]])-.12)
    lookup=np.full(grid.shape,-1,int);lookup[ref[:,1],ref[:,0]]=np.arange(len(xy))
    supported=[]
    for step in np.arange(0,max(0.,float(np.max(end-begin)))+res*.3,res*.3):
        dist=begin+step; active=(begin>=.35)&(dist<=end)
        if not active.any():continue
        dirs=direction[active]
        ij=np.floor((sensor[:2]+dirs*dist[active,None]-origin)/res).astype(int)
        inside=(ij[:,0]>=0)&(ij[:,1]>=0)&(ij[:,0]<grid.shape[1])&(ij[:,1]<grid.shape[0])
        ids=lookup[ij[inside,1],ij[inside,0]];ok=ids>=0
        ids=ids[ok];dirs=dirs[inside][ok];delta=xy[ids]-sensor[:2]
        along=np.sum(delta*dirs,axis=1)
        exact=(abs(delta[:,0]*dirs[:,1]-delta[:,1]*dirs[:,0])<=.42*res)&(along>=begin[active][inside][ok])&(along<=end[active][inside][ok])
        supported.extend(ids[exact])
    confirmed=np.unique(np.asarray(supported,int))
    return (np.setdiff1d(confirmed,veto_idx) if veto_neighbours else confirmed),veto_idx
