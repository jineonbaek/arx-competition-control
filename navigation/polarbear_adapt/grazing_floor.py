"""Evidence for low head speckle; measured distances and grids are in odom."""
from collections import deque
import numpy as np
from scipy.ndimage import convolve

class GrazingFloor:
    def __init__(self):
        self.frames=deque(maxlen=96)
        self.verified=np.empty((0,2),int)
        self.streaks={}

    def classify(self,t,points,height,nrays):
        rejected=np.zeros(len(points),bool)
        self.verified=np.empty((0,2),int)
        low=(height>.022)&(height<=.055)
        ray=np.arange(len(points))<nrays
        def keys(mask):
            return np.unique(np.floor(points[mask,:2]/.05).astype(int),axis=0)
        if self.frames and t-self.frames[-1][0]>1.5:
            self.frames.clear();self.streaks.clear()
        while self.frames and t-self.frames[0][0]>10:self.frames.popleft()
        self.frames.append((t,keys((abs(height)<=.022)&ray),keys(low&ray),keys((height>.055)&(height<=.22))))
        _,floor_now,low_now,_=self.frames[-1]
        floor_set={tuple(k) for k in floor_now}
        self.streaks={k:v for k,v in self.streaks.items() if t-v[1]<=2.}
        for k in floor_set:self.streaks.pop(k,None)
        for coord in low_now:
            k=tuple(coord)
            if k not in floor_set:self.streaks[k]=(self.streaks.get(k,(0,0))[0]+1,t)
        if len(self.frames)<5:return rejected
        f,e,vertical=[np.vstack([v[i] for v in self.frames]) for i in (1,2,3)]
        q=keys(low)
        if not len(f) or not len(e):return rejected
        allkeys=np.vstack((f,e,q,vertical));origin=allkeys.min(0)-2;shape=allkeys.max(0)-origin+3
        def raster(keys):
            a=np.zeros(shape,np.int32)
            if len(keys):np.add.at(a,tuple((keys-origin).T),1)
            return a
        F,E=raster(f),raster(e)
        small=np.ones((3,3),np.int32);patch=np.ones((5,5),np.int32)
        fs=convolve(F,small,mode='constant');es=convolve(E,small,mode='constant')
        wide=convolve(F,patch,mode='constant')
        side=convolve(raster(vertical),small,mode='constant')>0
        # Each count per cell is a distinct scan. An elevated patch or repeated
        # raised point at a single XY location remains occupied.
        ok=(F>=1)&(fs>=8)&(wide>=16)&(fs>=2*es)&~side&~((E>=3)&(E>F))
        persistent=np.asarray([k for k,(n,_) in self.streaks.items() if n>=2],int).reshape(-1,2)-origin
        persistent=persistent[np.all((persistent>=0)&(persistent<shape),axis=1)]
        if len(persistent):ok[tuple(persistent.T)]=False
        if low.any():rejected[low]=ok[tuple((np.floor(points[low,:2]/.05).astype(int)-origin).T)]
        self.verified=np.argwhere(ok&(F>=2)&(E>0))+origin
        return rejected
