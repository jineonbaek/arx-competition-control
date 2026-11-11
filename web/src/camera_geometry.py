"""Pure RealSense geometry helpers; importing this does not open a camera."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def camera_intrinsics(k):
    vendor=str(ROOT/'vision/vendor-python')
    if vendor not in sys.path:sys.path.insert(0,vendor)
    import pyrealsense2 as rs
    intr=rs.intrinsics()
    intr.width,intr.height=k['width'],k['height']
    intr.fx,intr.fy,intr.ppx,intr.ppy=k['fx'],k['fy'],k['cx'],k['cy']
    intr.model=getattr(rs.distortion,k['model'].split('.')[-1])
    intr.coeffs=k['coeffs']
    return rs, intr
