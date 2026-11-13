"""Conservative 3-D inclusion for measured return bounds in the fixed chassis."""
import numpy as np


class ChassisSolid:
    def __init__(self,path):
        with np.load(path,allow_pickle=False) as data:
            if str(data['link'])!='base_link':raise ValueError('Expected base_link interior')
            grid=data['solid']
            self.pitch=float(data['pitch'])
            self.lower=data['lower']
            self.shape=np.array(grid.shape)
            self.deck=float(data['deck'])
        # Integral volume of non-interior cells, with a zero border.
        self.empty=np.pad((~grid).astype(np.int32),((1,0),)*3)
        for axis in range(3):
            np.cumsum(self.empty,axis=axis,dtype=np.int32,out=self.empty)

    def contains_bounds(self,low,high,rotation,translation):
        # Transform all possible historic returns using the stamped odom->body pose.
        # Bounding the rotated box in body axes also protects mixed boundary voxels.
        middle=((low+high)*.5-translation)@rotation
        half=(high-low)*.5@np.abs(rotation)
        lo=np.floor((middle-half)/self.pitch+.5).astype(int)-self.lower
        hi=np.floor((middle+half)/self.pitch+.5).astype(int)-self.lower+1
        valid=(lo>=0).all(1)&(hi<=self.shape).all(1)&(middle[:,2]+half[:,2]<self.deck)
        a,b,c=lo[valid].T;x,y,z=hi[valid].T;q=self.empty
        count=(q[x,y,z]-q[a,y,z]-q[x,b,z]-q[x,y,c]+q[a,b,z]+q[a,y,c]+q[x,b,c]-q[a,b,c])
        valid[valid]=count==0
        return valid
