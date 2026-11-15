"""Navigation hit gate in the ground-referenced, gravity-aligned world frame.

Apply to actual measured XYZ before projecting to LaserScan. Clearing rays and
mapping clouds retain their original measurements and occlusion information.
"""
import numpy as np

MIN_HEIGHT=.03
MAX_HEIGHT=.40


def in_height_band(world_points):
    points=np.asarray(world_points)
    # Roundoff only: TF arithmetic and PointCloud2 store float32 coordinates.
    epsilon=1e-7
    return (np.isfinite(points[:,:3]).all(axis=1)
            & (points[:,2]>=MIN_HEIGHT-epsilon)
            & (points[:,2]<=MAX_HEIGHT+epsilon))
