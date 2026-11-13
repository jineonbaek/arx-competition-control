import ctypes as ct
from pathlib import Path
import weakref
import numpy as np

lib=ct.CDLL(str(Path(__file__).with_name('libfast_rays.so')))
ints=np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS')
floats=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
lib.new_history.restype=ct.c_void_p
lib.free_history.argtypes=[ct.c_void_p]
lib.update_bounds.argtypes=[ct.c_void_p,ct.c_int,ints,floats]
lib.erase_keys.argtypes=[ct.c_void_p,ct.c_int,ints]
lib.query_rays.argtypes=[ct.c_void_p,floats,ct.c_int,floats,ints,ct.c_int]
lib.query_rays.restype=ct.c_int

class FastRays:
    def __init__(self):
        self.handle=lib.new_history()
        weakref.finalize(self,lib.free_history,self.handle)
        self.buf=np.empty((1024,3),np.int32)

    def set(self,keys,bounds):
        lib.update_bounds(self.handle,len(keys),np.ascontiguousarray(keys,dtype=np.int32),np.ascontiguousarray(bounds,dtype=np.float64))

    def erase(self,keys):
        if keys:lib.erase_keys(self.handle,len(keys),np.ascontiguousarray(list(keys),dtype=np.int32))

    def crossed(self,origin,rays):
        o=np.ascontiguousarray(origin,dtype=np.float64)
        p=np.ascontiguousarray(rays,dtype=np.float64)
        while True:
            n=lib.query_rays(self.handle,o,len(p),p,self.buf,len(self.buf))
            if n>=0:return set(map(tuple,self.buf[:n]))
            self.buf=np.empty((len(self.buf)*2,3),np.int32)
