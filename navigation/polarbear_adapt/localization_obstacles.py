"""Obstacle evidence for the saved competition map's initial scan match."""
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt


# Bounds follow the screenshot edges in the competition map coordinate frame.
COMPETITION_FRAME = '7aa8506272a6422595877381d57483aa'
COMPETITION_BOUNDS = (-12.70, -4.10, 1.44, 3.98)
MIN_MATCH_POINTS = 25


class ObstacleRegion:
    def __init__(self, bounds, shape, resolution, origin):
        xmin, ymin, xmax, ymax = bounds
        height, width = shape
        x = origin[0] + (np.arange(width) + .5) * resolution
        y = origin[1] + (np.arange(height) + .5) * resolution
        if xmin < origin[0] - 1e-6 or ymin < origin[1] - 1e-6 or (
            xmax > origin[0] + width * resolution + 1e-6
            or ymax > origin[1] + height * resolution + 1e-6
        ):
            raise ValueError('比赛地图未完整覆盖截图配准区域')
        self.mask = ((y >= ymin) & (y < ymax))[:, None] & ((x >= xmin) & (x < xmax))[None, :]
        self.bounds = bounds

    def distances(self, occupied, resolution):
        landmarks = occupied & self.mask
        if np.count_nonzero(landmarks) < MIN_MATCH_POINTS:
            raise ValueError('截图配准区域内的地图障碍不足')
        return distance_transform_edt(~landmarks) * resolution

    def sample(self, distances, indices):
        height, width = distances.shape
        x, y = indices[..., 0], indices[..., 1]
        inside = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        x, y = x.clip(0, width - 1), y.clip(0, height - 1)
        return np.where(inside & self.mask[y, x], distances[y, x], np.nan)


def selected_region(map_path, shape, resolution, origin):
    metadata = Path(map_path).resolve().parent / 'coordinate-frame.json'
    if not metadata.is_file():
        return None
    frame = json.loads(metadata.read_text())
    if frame['coordinate_id'] != COMPETITION_FRAME:
        return None
    return ObstacleRegion(COMPETITION_BOUNDS, shape, resolution, origin)


def scores(errors, minimum=MIN_MATCH_POINTS):
    valid = np.isfinite(errors)
    count = valid.sum(axis=-1)
    total = np.where(valid, np.minimum(errors, .50), 0.).sum(axis=-1)
    # Keep the objective finite during refinement, including outside the ROI.
    return np.where(count >= minimum, total / np.maximum(count, 1), 1.)


def evidence(errors, restricted):
    if restricted:
        errors = errors[np.isfinite(errors)]
        if len(errors) < MIN_MATCH_POINTS:
            return None
    return dict(score_m=float(np.minimum(errors, .50).mean()),
                median_m=float(np.median(errors)),
                within_10cm=float(np.mean(errors <= .10)),
                used_points=len(errors), matched_points=int(np.count_nonzero(errors <= .10)))


def rank_by_region_evidence(candidates):
    """Prevent a small matching subset from outranking a well-supported pose."""
    strongest = max((candidate['matched_points'] for candidate in candidates), default=0)
    if strongest:
        for candidate in candidates:
            candidate['mean_roi_error_m'] = candidate['score_m']
            candidate['score_m'] += .10 * (1. - candidate['matched_points'] / strongest)
