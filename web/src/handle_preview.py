"""Estimate the front face of a manually selected handle in a frozen base_link cloud."""
import numpy as np


def handle_line(xyz, base_from_color, roi, clearance_mm):
    if type(clearance_mm) not in (int, float) or not 1 <= clearance_mm <= 200:
        raise ValueError("预抓距离范围为 1～200 mm")
    x, y, right, bottom = roi
    patch = xyz[y:bottom, x:right]
    rows, cols = np.nonzero(np.isfinite(patch).all(axis=2) & (patch[:,:,2] > 0))
    if len(rows) < max(24, patch.shape[0]*patch.shape[1]*.25):
        raise ValueError("有效深度不足，请紧框提手前沿实体，避开孔洞和背景")
    points = (patch[rows,cols] @ base_from_color[:3,:3].T + base_from_color[:3,3])*1000
    # A frontal surface should form one narrow layer along vehicle X.
    xs = np.sort(points[:,0])
    ends = np.searchsorted(xs, xs + 10, side="right")
    start = int(np.argmax(ends - np.arange(len(xs))))
    keep = (points[:,0] >= xs[start]) & (points[:,0] <= xs[start]+10)
    if keep.sum() < max(24, .55*len(points)):
        raise ValueError("框内深度分层或表面倾斜，请重新框选正对夹爪的提手前沿")
    half_x = (right-x)/2
    half_y = (bottom-y)/2
    for left_half in (True, False):
        for upper_half in (True, False):
            if np.count_nonzero(keep & ((cols < half_x)==left_half) & ((rows < half_y)==upper_half)) < 3:
                raise ValueError("前沿支持点未覆盖选框四周，请重新框选")
    surface = points[keep]
    # Midrange of lateral/vertical extent avoids bias from missing central depth.
    lo, hi = np.percentile(surface, [5,95], axis=0)
    end = (lo + hi)/2
    end[0] = np.median(surface[:,0])
    pre = end - [clearance_mm, 0, 0]
    return dict(frame="base_link", unit="mm", feature_mm=end.tolist(),
                pre_mm=pre.tolist(), valid=len(points), inliers=int(keep.sum()))
