# Dual-camera offline replay

Read-only archived-frame analysis: D435 detects a single red opening and estimates a coarse plane and center from a sparse base-frame point cloud. D405 estimates a bounded vertical offset when both rim side edges and the held battery front are visible; a top-clipped rim is accepted when sufficient visible side-edge support remains.

The result is always marked `executable: false`. This module contains no robot, ROS, CAN, or motion-control interface. It does not implement lateral correction, battery tilt, calibrated insertion, full collision checking, or a live deployment control path. It does not validate robot or scene state. Use a separate, verified control workflow for hardware.

## Input and calibration

Each input must be a capture view JSON with `image` (JPEG data URI), `points` (base-frame XYZ plus color, sampled every fourth image pixel), `base_from_color`, `width`, `height`, and `time`. No images, point clouds, credentials, or field captures are committed to this repository.

`intrinsics.json` contains example values recorded for one 640×480 setup. Supply `--intrinsics` with verified current camera parameters for another setup. The archived-view format lacks camera serial and explicit distortion data; this approximate reconstruction ignores inverse-Brown distortion and cannot authenticate camera identity. Output from sparse reconstruction is not a calibrated measurement. In this captured setup, D435 extrinsics were a CAD estimate and D405 extrinsics were an offline candidate; neither is a physical accuracy certificate.

Example:

```sh
python -m unittest -v test_offline
python offline.py /path/to/d435-view.json /path/to/d405-view.json --intrinsics /path/to/intrinsics.json > replay.json
```

Dependencies: Python 3, NumPy, OpenCV. The two frame timestamps are reported; a difference above 0.2 s is marked unpaired. These archived views are not suitable for current robot motion, even when timestamps are paired. Collect fresh raw synchronized frames and verify calibration, payload attachment, reachable path, and environmental clearance before considering live testing.
