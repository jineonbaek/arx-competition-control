"""Bound MPPI's path before the next unobserved via, without changing the mission."""
import math
import numpy as np

def controller_route(route,vias):
    route=np.asarray(route,dtype=float)
    if route.ndim!=2 or route.shape[1]!=3 or not len(route) or not np.isfinite(route).all():raise ValueError('invalid_controller_route')
    if not vias:return route.copy()
    via=vias[0];target=np.asarray(via['pose'][:2]);radius=via['pass_radius_m']
    for i,a in enumerate(route):
        b=route[min(i+1,len(route)-1)];d=b[:2]-a[:2];den=d@d
        u=0. if den<1e-18 else float(np.clip((target-a[:2])@d/den,0.,1.))
        xy=a[:2]+u*d
        if np.linalg.norm(xy-target)<=radius+1e-9:
            yaw=a[2]+u*math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
            # Original prefix then a short separately audited connector to the via
            # centre. No arrow-heading stop is imposed on continuous via points.
            return np.vstack((route[:i+1],np.r_[xy,yaw],np.r_[target,yaw]))
    raise ValueError('via_not_on_path')
