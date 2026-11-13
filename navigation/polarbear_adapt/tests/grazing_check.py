import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from grazing_floor import GrazingFloor
# Broad scattered elevated observations over repeatedly measured flat floor.
xy=np.array([[x*.05+.025,y*.05+.025] for x in range(-2,3) for y in range(-2,3)])
f=np.c_[xy,np.zeros(len(xy))]
def scan(g,t,pts):
    p=np.array(pts).reshape(-1,3)
    return g.classify(t,p,p[:,2],len(p))
g=GrazingFloor()
for i in range(7):scan(g,.1*i,f)
target=np.array([[.025,.025,.038]])
assert scan(g,.8,np.vstack([f,target]))[-1]
assert (g.verified==[0,0]).all(axis=1).any()
# An actual vertical side at the candidate vetoes suppression.
assert not scan(g,.9,np.vstack((f,target,[.026,.026,.11])))[-2]
# A newly raised small object with repeated returns is protected even when its
# column had substantial historical floor support before it appeared.
g=GrazingFloor()
for i in range(7):scan(g,.1*i,f)
scan(g,.8,target);assert not scan(g,.9,target)[-1]
print('PASS: repeat-hit floor, independent upright veto, emerging low object protection')
