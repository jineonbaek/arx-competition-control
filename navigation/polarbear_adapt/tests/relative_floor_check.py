"""Floor-only occupancy can be removed without clearing a cone base or a higher return at the same XY."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from shared_voxels import World,SIZE
w=World();rng=np.random.default_rng(17)
xy=np.array([(x,y) for x in np.arange(-1.6,1.65,.08) for y in np.arange(-1.6,1.65,.08)])
height=lambda p:.012*p[:,0]-.01*p[:,1]+.007
plane=np.c_[xy,height(xy)+rng.normal(0,.0015,len(xy))]
target=np.array([[.57,.33],[.61,.34],[.575,.375],[.605,.365]])
flat=np.c_[target,height(target)]
upper=flat+[0.,0.,.61]
cone=flat+[.42,0.,.036]
table=upper
empty=np.empty((0,3));bottom=np.array([0,0,.23]);head=np.array([0,0,1.19279])
# Establish genuine historical hits without floor support; disable rays during this phase.
for i in range(2):
    w.update('bottom',1+i*.1,bottom,empty,np.vstack([flat,upper,cone]))
floor_keys={tuple(k) for k in np.floor(flat/SIZE).astype(int)}
upper_keys={tuple(k) for k in np.floor(upper/SIZE).astype(int)}
cone_keys={tuple(k) for k in np.floor(cone/SIZE).astype(int)}
assert floor_keys&set(w.cells) and upper_keys&set(w.cells) and cone_keys&set(w.cells)
# Only floor points as seeds: no crossing ray here to artificially clear the old cell.
for i in range(12):
    t=2+i*.105
    w.update('head',t,head,empty,np.vstack((flat,table,cone)),ground=np.vstack((plane,flat)))
assert not floor_keys&set(w.cells) and w.floor_removed>0
assert upper_keys&set(w.cells) and cone_keys&set(w.cells)
report=w.inspect(target[0],3.3)
assert report['local_floor_z'] is not None and any('head' in row['sources'] and 'bottom' in row['sources'] for row in report['voxels'])
assert any(row['height_above_floor'][1]>.55 for row in report['voxels'])
# A 3.5 cm low face can extend an existing column, but flat elevated speckle alone cannot create a ring.
new=cone+[0.,.4,0.]
for i in range(4):
    w.update('head',3.4+i*.1,head,empty,empty,ground=plane,extra=new)
assert not {tuple(k) for k in np.floor(new/SIZE).astype(int)}&set(w.cells)
for i in range(4):
    w.update('head',3.9+i*.1,head,empty,new+[0,0,.080],ground=plane,extra=new)
assert {tuple(k) for k in np.floor(new/SIZE).astype(int)}&set(w.cells)
# No evidence at another location -> no invented local surface or ground-only clearing.
assert np.isnan(w.floor.height(np.array([[10.,10.]]))[0])
print('PASS: low historical floor removed; table above it and 3.5cm cone preserved; dual-source click attribution; unsupported region unchanged.')
