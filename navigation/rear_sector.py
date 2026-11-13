"""Bottom upright Livox optical coordinates; +X forward and +Y left."""
import numpy as np

REAR_EDGE = np.deg2rad(137.5)


def rear_points(points):
    angles = np.arctan2(points[:, 1], points[:, 0])
    return np.abs(angles) >= REAR_EDGE


def mask_scan(ranges, start, increment):
    angles = start + np.arange(len(ranges)) * increment
    # Scan angles are float32 on the wire, including the two boundary bins.
    wrapped = (angles + np.pi) % (2 * np.pi) - np.pi
    values = np.asarray(ranges, dtype=float).copy()
    values[np.abs(wrapped) >= REAR_EDGE - 2e-7] = np.nan
    return values.tolist()
