"""Revalidate only the remaining original geometry against the current grid."""
import numpy as np
from continuous_track import OffPath,projected_progress

def safe_suffix(route,pose,grid):
    # Geometric projection is not proof that a nearby later loop was traversed.
    # Audit the full accepted plan as well; ambiguity remains fail-closed.
    if grid.along(route)['first'] is not None:return None
    try:index,projection=projected_progress(route,pose)
    except OffPath:return None
    remaining=np.vstack([pose,projection,route[index+1:]])
    # The accepted tail is a subset of the full path certified above. Only
    # the short new connector needs another query; do not re-audit a moving
    # full suffix whose cache key would change at every odometry update.
    return remaining if grid.along([pose,projection])['first'] is None else None
