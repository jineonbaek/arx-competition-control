"""Display-only clipping; never changes pose, depth, or grasp geometry."""
import cv2
import numpy as np


def pixel(point):
    p = np.asarray(point, dtype=float)
    # Extreme optical projections have no useful display precision.
    if p.shape != (2,) or not np.isfinite(p).all() or np.max(np.abs(p)) > 1e15:
        return None
    return [int(round(float(v))) for v in p]


def visible(point, image):
    return point is not None and 0 <= point[0] < image.shape[1] and 0 <= point[1] < image.shape[0]


def draw_line(image, start, end, color, thickness, *, arrow=False):
    """Clip in floating point before handing bounded integers to OpenCV."""
    a, b = pixel(start), pixel(end)
    h, w = image.shape[:2]
    if a is None or b is None or not h or not w:
        return None
    x, y = map(float, a)
    dx, dy = float(b[0])-x, float(b[1])-y
    low, high = 0., 1.
    for p, q in ((-dx, x), (dx, w-1-x), (-dy, y), (dy, h-1-y)):
        if p == 0:
            if q < 0:
                return None
            continue
        ratio = q/p
        if p < 0:
            low = max(low, ratio)
        else:
            high = min(high, ratio)
        if low > high:
            return None
    def endpoint(t):
        return (max(0, min(w-1, round(x+t*dx))), max(0, min(h-1, round(y+t*dy))))
    first, last = endpoint(low), endpoint(high)
    if arrow and visible(b, image):
        cv2.arrowedLine(image, first, last, color, thickness, cv2.LINE_AA, tipLength=.15)
    else:
        cv2.line(image, first, last, color, thickness, cv2.LINE_AA)
    return [list(first), list(last)]
