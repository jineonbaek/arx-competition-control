"""Conservative resolution change for the planning copy, never saved-map data."""
from array import array
import copy
import math
import numpy as np

COSTMAP_RESOLUTION = .02


def _snap(value):
    nearest=round(value)
    return nearest if math.isclose(value,nearest,rel_tol=1e-7,abs_tol=1e-8) else value


def _resize_axis(encoded, source_resolution, target_resolution, axis):
    source_size=encoded.shape[axis]
    target_size=math.ceil(_snap(source_size*source_resolution/target_resolution))
    shape=list(encoded.shape); shape[axis]=target_size
    out=np.empty(shape,dtype=np.int16)
    for index in range(target_size):
        begin=math.floor(_snap(index*target_resolution/source_resolution))
        end=math.ceil(_snap((index+1)*target_resolution/source_resolution))
        a,b=max(0,begin),min(source_size,end)
        src=[slice(None)]*2; dst=[slice(None)]*2
        src[axis]=slice(a,b); dst[axis]=index
        if a>=b:
            out[tuple(dst)]=1  # unknown beyond the original extent
            continue
        values=encoded[tuple(src)].max(axis=axis)
        if begin<0 or end>source_size:values=np.maximum(values,1)
        out[tuple(dst)]=values
    return out


def resample_occupancy(grid,source_resolution,target_resolution=COSTMAP_RESOLUTION):
    """Every positively overlapping occupied cell survives; unknown beats free.

    Encoded priorities: free=0, unknown=1, occupancy1..100=2..101.
    The maximum over all source cells intersecting an output cell therefore
    cannot create free space by interpolating occupied or unknown values.
    """
    if not all(math.isfinite(r) and r>0 for r in (source_resolution,target_resolution)):
        raise ValueError('Resolution must be positive and finite')
    source=np.asarray(grid)
    if (source.ndim!=2 or 0 in source.shape or not np.isfinite(source).all()
            or (source<-1).any() or (source>100).any()
            or not np.equal(source,np.floor(source)).all()):
        raise ValueError('Expected nonempty 2D OccupancyGrid values -1..100')
    if math.isclose(source_resolution,target_resolution,rel_tol=1e-7,abs_tol=1e-10):
        return source.astype(np.int8,copy=True)
    encoded=np.where(source<0,1,np.where(source>0,source+1,0)).astype(np.int16)
    encoded=_resize_axis(encoded,source_resolution,target_resolution,1)
    encoded=_resize_axis(encoded,source_resolution,target_resolution,0)
    return np.where(encoded==1,-1,np.where(encoded>1,encoded-1,0)).astype(np.int8)


def planning_copy(message,target_resolution=COSTMAP_RESOLUTION):
    """Preserve map pose/header and modify a deep copy only; ROS-independent."""
    info=message.info
    source=np.asarray(message.data,dtype=np.int16).reshape(info.height,info.width)
    result=resample_occupancy(source,info.resolution,target_resolution)
    output=copy.deepcopy(message)
    output.info.resolution=target_resolution
    output.info.height,output.info.width=result.shape
    values=array('b'); values.frombytes(result.tobytes()); output.data=values
    return output
