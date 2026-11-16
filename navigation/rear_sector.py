"""Per-lidar azimuth masks in native optical axes.

Front: +X vehicle-forward, rear85 blocked. Rear: +X vehicle-left,
+Y vehicle-back, so vehicle-forward70 is centered on native -Y.
Directions are measured from each sensor, not from C_base.
"""
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


REAR_FORWARD_HALF = np.deg2rad(35.)

def rear_forward_points(points):
    angles = np.arctan2(points[:,1],points[:,0])
    delta = (angles+np.pi/2+np.pi)%(2*np.pi)-np.pi
    return np.abs(delta)<=REAR_FORWARD_HALF+2e-7

def mask_rear_forward_scan(ranges,start,increment):
    angles=start+np.arange(len(ranges))*increment
    delta=(angles+np.pi/2+np.pi)%(2*np.pi)-np.pi
    values=np.asarray(ranges,dtype=float).copy()
    values[np.abs(delta)<=REAR_FORWARD_HALF+2e-7]=np.nan
    return values.tolist()
