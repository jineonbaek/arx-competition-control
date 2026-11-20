"""Publish the latest D405 alignment evidence for the placement page."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import uuid
import threading

import cv2
import numpy as np
from d405_legacy_overlay import annotate

_VISUAL_LOCK=threading.RLock()


def _atomic_bytes(path, payload):
    temporary = path.with_name(f"{path.name}.tmp-{uuid.uuid4().hex}")
    temporary.write_bytes(payload)
    os.replace(temporary, path)


def _point_pairs(measurement):
    rows = measurement.get("rim_rows_px", [])
    inner = measurement.get("rim_inner_px", [])
    left = [[float(x[0]), float(y)] for y, x in zip(rows, inner)]
    right = [[float(x[1]), float(y)] for y, x in zip(rows, inner)]
    battery = [[float(x), float(y)] for y, x in measurement.get("battery_front_px", [])]
    return left + right, battery


def write_horizontal_overlay(frame, measurement, output_dir):
    """Atomically replace the image and matching evidence metadata."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    rgb = np.asarray(frame["rgb"])
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("Invalid D405 RGB image")
    with _VISUAL_LOCK:
        # Static captures use fresh observed lines; live trackers remain in the
        # restored module, but an old held contour cannot label a new snapshot.
        image,visual=annotate(rgb,'d405',xyz=frame.get('xyz'),base_from_color=frame.get('base_from_color'),
            face_observation=frame.get('battery_face_observation'))
    ready = measurement.get("status") == "visual_guidance_only"
    rim_points, battery_points = _point_pairs(measurement) if ready else ([], [])
    if not ready:
        hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
        red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60)).astype(np.uint8)
        count,labels,stats,_=cv2.connectedComponentsWithStats(red,8)
        candidates=[i for i in range(1,count) if stats[i,4]>=300]
        if candidates:
            mask=(labels==max(candidates,key=lambda i:stats[i,4])).astype(np.uint8)
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            contour=max(contours,key=cv2.contourArea)
            # Follow visible pixels of the open rim; do not fill hidden edges.
            if not visual.get('rim_outline'):
                cv2.drawContours(image,[contour],-1,(255,255,255),4,cv2.LINE_8)
                cv2.drawContours(image,[contour],-1,(0,0,255),2,cv2.LINE_8)
            rim_points=contour[::max(1,len(contour)//120),0,:].astype(float).tolist()
            if visual.get('battery',{}).get('status')!='perspective_four_edge_fit':
                cv2.putText(image,'RIM VISIBLE / BATTERY EDGE UNCONFIRMED',(8,image.shape[0]-10),
                    cv2.FONT_HERSHEY_SIMPLEX,.42,(0,255,255),1,cv2.LINE_AA)

    if ready:
        rows = measurement["rim_rows_px"]
        inner = measurement["rim_inner_px"]
        for y, pair in zip(rows, inner):
            for x in pair:
                cv2.circle(image, (round(x), round(y)), 3, (0, 0, 255), -1, cv2.LINE_8)
        battery = np.asarray(measurement["battery_front_px"], dtype=float)
        stride = max(1, len(battery) // 80)
        for y, x in battery[::stride]:
            cv2.circle(image, (round(x), round(y)), 2, (255, 0, 255), -1, cv2.LINE_8)
        centre = np.median(battery, axis=0)
        direction = 1 if float(measurement["step_mm"]) >= 0 else -1
        start = (round(centre[1]), round(centre[0] - 12))
        end = (start[0] + direction * 32, start[1])
        cv2.arrowedLine(image, start, end, (0, 255, 255), 4, cv2.LINE_8, tipLength=.3)
        label = f"Y {measurement['raw_error_mm']:+.1f} mm / step {measurement['step_mm']:+.1f}"
        cv2.putText(image, label, (8, image.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    .42, (0, 255, 255), 1, cv2.LINE_AA)

    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 100])
    if not ok:
        raise RuntimeError("D405 overlay JPEG encoding failed")
    image_path = output / "d405-horizontal.jpg"
    metadata_path = output / "d405-horizontal.json"
    _atomic_bytes(image_path, encoded.tobytes())

    metadata = {
        "status": "ready" if ready else ("partial" if rim_points else "unavailable"),
        "reason": measurement.get("reason"),
        "frame_id": measurement.get("frame_id"),
        "image_time": measurement.get("image_time"),
        "calibration_sha256": measurement.get("calibration_sha256"),
        "image_file": "/camera-overlay/d405-horizontal.jpg",
        "rim": {"status": "visible" if ready else "visible_partial", "points": rim_points} if rim_points else {"status": "unavailable"},
        "battery_front": {"status": "visible", "points": battery_points} if ready else {"status": "unavailable"},
        "battery_visual": visual.get('battery'),
        "rim_visual": visual.get('rim_outline'),
        "visual_alignment": visual.get('alignment'),
        "visual_comparison": visual.get('comparison'),
        "visual_space": "image_pixels",
        "visual_executable": False,
        "target_offset_mm": measurement.get("target_offset_mm"),
        "raw_error_mm": measurement.get("raw_error_mm"),
        "step_mm": measurement.get("step_mm"),
        "attempt": measurement.get("attempt"),
        "cumulative_correction_mm": measurement.get("cumulative_correction_mm"),
        "converged": measurement.get("converged"),
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }
    payload = json.dumps(metadata, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode()
    _atomic_bytes(metadata_path, payload)
    return metadata
