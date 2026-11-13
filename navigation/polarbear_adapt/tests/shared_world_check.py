"""Cross-sensor persistence, true 3-D clearing, and track-owned footprint replacement."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from shared_voxels import World,SIZE

empty=np.empty((0,3));origin=np.array([0.,0.,.23])
def object_at(x,y,sz=.3):
    return np.array([[x+dx,y+dy,.15+z] for dx in np.arange(-sz/2,sz/2+.001,.05)
                     for dy in [-.06,0.,.06] for z in np.arange(0,.65,.075)])
def occupied(w,k):
    return k in w.cells
w=World();p=object_at(1.5,.0)
w.update('head',1.,origin,empty,p)
k=tuple(np.floor(p[len(p)//2]/SIZE).astype(int));assert occupied(w,k)
w.update('bottom',5.,origin,empty,empty)
assert occupied(w,k) and len(w.projection(20.))>0
center=(np.asarray(k)+.5)*SIZE
# Same XY but high beam, occluded finite ray, and single proper crossing all retain it.
for t in [5.1,5.2]:
    w.update('bottom',t,origin+np.array([0,0,1]),np.array([origin+2*(center-origin)+[0,0,1]]),empty)
    assert occupied(w,k)
occluded=origin+.6*(center-origin)
for t in [5.3,5.4]:
    w.update('bottom',t,origin,np.array([occluded,origin+2*(center-origin)]),empty)
assert occupied(w,k)
surface=p[len(p)//2].copy()
# Aim inside the XY cell at the actual return height.
surface[:2]=center[:2]
ray=np.array([origin+2*(surface-origin)])
w.update('bottom',5.5,origin,ray,empty);assert occupied(w,k)
w.update('bottom',5.6,origin,empty,surface[None,:]);assert occupied(w,k)
w.update('bottom',5.7,origin,ray,empty);assert occupied(w,k)
w.update('bottom',5.8,origin,ray,empty);assert not occupied(w,k)
w.update('head',5.65,origin,empty,p);assert not occupied(w,k)
# Include another background hit outside the mover. A brief lost association retains last occupancy.
w=World();wall=np.array([[2.,1.,.3],[2.05,1.,.3],[2.1,1.,.3]])
fixed_key=tuple(np.floor(wall[0]/SIZE).astype(int))
for i in range(22):
    person=object_at(1.+i*.065,.25)
    # Actual through-rays cross some, but not all, of the mover's old voxels.
    old_keys=[k for k,c in w.cells.items() if c.owner and (k[0]+.5)*SIZE[0] < person[:,0].min()-.03]
    rays=np.array([origin+2*((np.asarray(k)+.5)*SIZE-origin) for k in old_keys
                   if k[2]%3==0]).reshape(-1,3)
    w.update(['head','bottom'][i%2],10+i*.1,origin,rays,np.vstack((person,wall)))
    assert fixed_key in w.cells
assert w.track_removed>0
old=[k for k in w.cells if abs((k[1]+.5)*SIZE[1]-.25)<.15 and (k[0]+.5)*SIZE[0]<2.]
assert not old,old
before=len(w.cells)
w.update('bottom',15.,origin,empty,empty);assert len(w.cells)==before
print('PASS: missed scans/head exit retain cells; occlusion and high ray retain; two bottom rays erase head voxel; moving object leaves no old footprint; separate static hits retained; last known location retained on loss')

# The old centre-distance test misses this measured return near the top face.
from shared_voxels import Cell, measured_crossings, ray_crossings
k=(30,0,2)
ctr=(np.array(k)+.5)*SIZE
z=(k[2]+1)*SIZE[2]-.001
o=np.array([0.,ctr[1],z]); passing=np.array([[3.,ctr[1],z]])
w=World();w.cells[k]=Cell(0.,0,zmin=z,zmax=z)
assert not len(ray_crossings(ctr[None,:],o,passing))
assert measured_crossings(ctr[None,:],np.array([[z,z]]),o,passing).tolist()==[0]
w.update('head',1.,o,passing,empty)
w.update('bottom',2.1,o,passing,empty)
assert k not in w.cells
w.cells[k]=Cell(3.,0,zmin=z,zmax=z)
w.update('head',4.,o,passing,empty)
w.update('bottom',4.4,o,np.array([[ctr[0],ctr[1],z]]),empty)
w.update('head',5.,o,passing,empty)
assert k in w.cells
w.update('head',7.2,o,passing,empty)
assert w.inspect(ctr[:2],7.2)['voxels'][0]['recent_clear_frames']==1
print('PASS: off-centre measured height, sparse crossings, raw endpoint veto and evidence expiry')

# Two finite beams at different heights revisit a tall voxel. The higher
# sample alone must not erase the lower return. Both can be cleared over time.
w=World()
k=(30,0,2)
xy=(np.array(k[:2])+.5)*SIZE[:2]
low, high=.156,.219
w.cells[k]=Cell(0.,0,zmin=low,zmax=high)
for t,z in [(1.,low+.013),(1.1,low+.013),(1.2,high-.013),(1.3,high-.013)]:
    # A sloped beam crosses the half-interval at this cell, but not both halves.
    direction=np.array([1.,0.,.67])
    centre=np.r_[xy,z]
    o=centre-direction
    w.update('head' if z<.18 else 'bottom',t,o,(centre+direction)[None,:],empty)
    assert (k in w.cells)==(t<1.3),(t,w.cells.get(k))
print('PASS: lower and upper evidence can come from different sensors and frames')

w=World()
for i in range(18):
    w.update('bottom',10+i*.1,origin,empty,np.vstack((object_at(1+i*.065,.25),wall)))
assert w.track_removed>0 and fixed_key in w.cells
assert not any(k[1]==5 and (k[0]+.5)*SIZE[0]<1.65 for k in w.cells)
print('PASS: coherent mover clears its own trail without synthetic crossing rays')
