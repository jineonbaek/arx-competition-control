"""Live, full-footprint near-horizon proof for accepted geometry. No cached free space."""
import numpy as np
from path_audit import AuditGrid
from continuous_track import projected_progress,OffPath
from mppi_trial.recovery import upcoming_route
from via_tracking import controller_route

def live_prefix(route,map_pose,odom_pose,global_grid,local_grid,vias):
    bounded=controller_route(route,vias)
    try:index,projection=projected_progress(bounded,np.asarray(map_pose))
    except OffPath as exc:raise ValueError("replan_blocked accepted_path_off_track "+str(exc)) from exc
    tail=np.vstack((map_pose,projection,bounded[index+1:]))
    prefix=upcoming_route(tail,global_grid.radius,distance=.6)
    yaw=map_pose[2]-odom_pose[2];c,s=np.cos(yaw),np.sin(yaw)
    local=prefix.copy();local[:,:2]=(prefix[:,:2]-map_pose[:2])@np.array([[c,-s],[s,c]])+odom_pose[:2];local[:,2]-=yaw
    proofs=[]
    for frame,grid,path in [('map',global_grid,prefix),('odom',local_grid,local)]:
        # Bypass only background scheduling, never the original continuous
        # footprint checker, unknown cells, corner margin or resolution.
        result=AuditGrid.along(grid,path)
        if result['first'] is not None:raise ValueError(frame+'_sweep_blocked '+str(result['first']))
        proofs.append(result)
    return dict(prefix=prefix,proofs=proofs)
