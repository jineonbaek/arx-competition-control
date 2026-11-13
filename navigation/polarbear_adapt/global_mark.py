"""Evidence for the global obstacle layer; local avoidance receives every hit."""
import numpy as np
from scipy.spatial import cKDTree


class GlobalMarkFilter:
    def __init__(self):
        self.previous = None
        self.stamp = None

    def keep(self, level_points, odom_origin, stamp):
        # pb_lidar_level shares odom axes and translates with the sensor.
        near = np.linalg.norm(level_points[:, :2], axis=1) <= 2.0
        keep = ~near
        world = level_points[near] + odom_origin
        current = cKDTree(world) if len(world) else None
        if current is not None:
            # Retain spatially supported clusters immediately. Duplicate samples
            # at exactly the same coordinates are a single observation.
            unique, inverse = np.unique(world, axis=0, return_inverse=True)
            neighbours = cKDTree(unique).query(unique, k=2)[0][:, 1]
            supported = neighbours[inverse] <= 0.075
            if self.previous is not None and 0 < stamp - self.stamp <= 0.25:
                supported |= self.previous.query(world)[0] <= 0.08
            keep[near] = supported
        self.previous = current
        self.stamp = stamp
        return keep
