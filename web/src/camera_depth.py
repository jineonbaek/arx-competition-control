"""Lossless depth frames for camera consumers; zero always remains invalid."""
import math
import cv2
import numpy as np


def write_depth(out,key,token,raw,scale,intrinsics,aligned,frame_number,timestamp_ms):
    if raw.dtype!=np.uint16 or raw.ndim!=2 or not math.isfinite(scale) or scale<=0:
        raise ValueError("Expected a uint16 depth image and a positive finite metre scale")
    name=f"{key}-z-{token}.png";preview=f"{key}-z-{token}.jpg"
    # Fixed display range, never a per-frame distance normalization.
    display_max=.7 if key=="d405" else 4.
    grey=np.clip(raw.astype(np.float32)*scale/display_max*255,0,255).astype(np.uint8)
    image=cv2.applyColorMap(255-grey,cv2.COLORMAP_TURBO);image[raw==0]=0
    png_ok,png=cv2.imencode(".png",raw,[cv2.IMWRITE_PNG_COMPRESSION,1])
    jpeg_ok,jpeg=cv2.imencode(".jpg",image,[cv2.IMWRITE_JPEG_QUALITY,82])
    if not png_ok or not jpeg_ok:raise RuntimeError("Depth encoding failed")
    paths=(out/name,out/preview)
    paths[0].write_bytes(png.tobytes());paths[1].write_bytes(jpeg.tobytes())
    return dict(state="online",image_file=name,preview_file=preview,
                width=raw.shape[1],height=raw.shape[0],encoding="16UC1",
                scale_m_per_unit=float(scale),invalid_value=0,
                aligned_to_color=aligned,intrinsics=intrinsics,
                frame_number=frame_number,sensor_timestamp_ms=timestamp_ms,
                valid_fraction=float(np.count_nonzero(raw)/raw.size),
                preview_range_m=[0,display_max]),paths
