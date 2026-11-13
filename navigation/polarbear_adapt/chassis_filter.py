"""Self-return masks in base_link coordinates (metres)."""
import numpy as np
from base_envelope import BaseEnvelope,BASE_ENVELOPE_CONFIG


# XY midpoints of the paired finger meshes in the upright assembly at arm zero.
# The lift translates along Z, so the two rectangles apply at every height.
GRIPPER_CENTRES = ((0.374010, 0.2527827), (0.374010, -0.2472173))
GRIPPER_HALF_X = 0.105 / 2
GRIPPER_HALF_Y = 0.170 / 2


def gripper_columns(body_points):
    x, y = body_points[:, 0], body_points[:, 1]
    inside = np.zeros(len(x), dtype=bool)
    for cx, cy in GRIPPER_CENTRES:
        inside |= ((x >= cx - GRIPPER_HALF_X) & (x <= cx + GRIPPER_HALF_X)
                   & (y >= cy - GRIPPER_HALF_Y) & (y <= cy + GRIPPER_HALF_Y))
    return inside & np.isfinite(body_points).all(axis=1)


def bottom_right_weak_returns(body_points, reflectivity):
    """Bottom MID360S return signature measured on the right side, in base_link metres."""
    if reflectivity is None:
        return np.zeros(len(body_points), dtype=bool)
    inside = ((body_points >= (.039, -.382, .325)) &
              (body_points <= (.102, -.319, .510))).all(axis=1)
    return inside & (reflectivity == 1)


def bottom_left_self_returns(body_points):
    """Measured front-lidar return inside the left rear CAD outline."""
    x, y, z = body_points.T
    # Upper edge of the unpadded CAD outline in this small x interval.
    edge = np.interp(x, [-.395, -.105, -.090, .130], [.075, .280, .290, .380])
    return ((x >= -.110) & (x <= -.060) &
            (y >= .243) & (y <= .300) & (y <= edge - .002) &
            (z >= .239) & (z <= .318))


class ChassisFilter:
    def __init__(self, mesh_file):
        self.envelope=BaseEnvelope.load(BASE_ENVELOPE_CONFIG)
        with np.load(mesh_file) as mesh:
            rigid = mesh["rigid"]
            self.pitch = float(mesh["pitch"])
        # Footprint of the rigid material below the 183 mm mounting deck.
        deck = rigid[(rigid[:, 2] >= 0.035) & (rigid[:, 2] <= 0.183)]
        cells = np.rint(deck[:, :2] / self.pitch).astype(int)
        self.origin = cells.min(axis=0)
        cells -= self.origin
        self.grid = np.zeros(cells.max(axis=0) + 1, dtype=bool)
        self.grid[tuple(cells.T)] = True

    def contains(self, body_points):
        result = gripper_columns(body_points)
        if self.envelope is not None:
            result |= self.envelope.mask(body_points)
        # Enclose the two observed cable-height returns (0.419 and 0.463 m).
        rows = np.flatnonzero(np.isfinite(body_points).all(axis=1)
                              & (body_points[:, 2] >= 0.0358)
                              & (body_points[:, 2] <= 0.48))
        cells = np.rint(body_points[rows, :2] / self.pitch).astype(int) - self.origin
        inside = ((cells >= 0) & (cells < self.grid.shape)).all(axis=1)
        result[rows[inside]] |= self.grid[tuple(cells[inside].T)]
        return result
