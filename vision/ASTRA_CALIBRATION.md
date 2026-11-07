# Astra Pro RGB-D calibration (ROG)

## Current evidence

- Device serial: `17122520757`; reported firmware: `5.8.22`.
- Legacy Astra Pro depth USB ID: `2bc5:0403`; RGB is a separate UVC device.
- The pinned SDK's 120-byte `OBCameraParams` (property 14) returned all `FF`, interpreted as non-finite floats. Repeated reads before/after depth start and in SDK registration mode had the same result. This proves these reads are unusable; it does not prove the camera was never factory calibrated or that its flash is erased.
- No reusable Astra calibration was found in the inspected ROG camera directories. Neither FOV-derived intrinsics nor mechanical CAD are a measured RGB-D calibration.
- User owns an aluminum board marked `CC200-15-11.25`. Board geometry, marker dictionary and physical dimensions must be checked against the actual target or its manufacturer drawing before solving. The supplied 1688 product page was not readable from this session; do not infer dimensions or dictionary from its model name alone.

## Implemented source behavior

`web/src/astra_depth.py` first confirms native/unregistered mode, reads and validates factory data, and identifies the camera by serial. It never flashes firmware, writes calibration to the device, resets USB, or accesses CAN.

1. If a measured calibration file for this exact serial and 640x480 profile exists, validate it and project native depth into color coordinates in software.
2. Otherwise valid factory parameters can enable SDK registration only after the SDK reports mode 1. Factory distortion ordering follows the pinned ROS camera-info implementation.
3. Invalid/missing parameters or an invalid external file leave raw depth available and report `required` or `error`; they never claim registered depth. If native mode itself cannot be confirmed, fail capture rather than mislabel coordinates.

Calibration metadata contains its source/hash and a separate `pending_physical_check` flag. A loaded transform or SDK mode is not an accuracy test. The UI preserves that distinction. Depth still defaults OFF and YOLOE stays OFF.

Capture-source changes require a graceful camera-only process reload before use. They are not loaded into an already-running worker. Do not restart the robot drivers. D435 currently has an independent stop/stream fault: do not reinitialize it just to test Astra.

## Measured file contract

Path for this device: `/home/arx/arx-competition-control/vision/config/astra-17122520757.json`.
No file with invented calibration values is installed.

Required JSON keys:

| Key | Meaning |
| --- | --- |
| `serial` | Exact device serial string |
| `depth_size`, `color_size` | Both `[640, 480]`, matching the calibrated capture profiles |
| `depth_K`, `color_K` | Finite 3x3 pinhole matrices with positive focal lengths |
| `depth_D`, `color_D` | Five OpenCV coefficients `[k1, k2, p1, p2, k3]` |
| `depth_to_color_R` | Proper 3x3 rotation from the native depth optical frame to the RGB optical frame |
| `depth_to_color_translation_m` | Translation in **metres**, same direction as R |

Optical axes: x right, y down, z forward. Registered uint16 output is color-camera optical Z in millimetres; zero remains invalid. Projection uses a depth buffer to keep near surfaces, checks points behind the RGB camera, and does not dilate or fill holes. Raw depth is undistorted before projection. A depth/IR pixel offset, crop, or different mode must be measured/verified; do not silently treat raw IR coordinates as identical to delivered depth coordinates.

## Physical calibration steps still required

1. Confirm board square counts, square length, marker length and ArUco dictionary from the actual board/manufacturer. Inspect it in the existing RGB preview first.
2. Arrange exclusive Astra acquisition through the camera owner; capture native IR and RGB views of a **stationary** board. Never open another RGB pipeline beside the preview, or simultaneous independent OpenNI depth/IR owners. Check the printed pattern is visible in infrared.
3. Collect a varied set of sharp views at different image positions and tilts. Both sensors must see the same target pose. Discard moved/blurred frames and reserve views for validation.
4. Fit intrinsics/distortion and depth-to-color extrinsics using matched, identified target points. Verify the delivered depth-to-IR pixel relationship before exporting the depth calibration schema above. Save measured evidence and the target specification with the result.
5. Check held-out reprojection errors, RGB/depth boundaries across the image and at different distances, and independently measured object distances. Report actual errors; do not invent an acceptance result or replace raw distance calibration with color registration.
6. Only then consider robot-frame hand-eye calibration. No calibration step here sends robot motion.

A small board is not automatically unsuitable: actual visible corner/marker size and overlap determine whether it can be detected in both sensors. RGB and Astra IR/depth do not have proven hardware synchronization; aligned pixel geometry does not synchronize moving scenes.

## Software verification

```bash
cd /home/arx/arx-competition-control
python3 -m unittest discover -s web/tests -p 'test_astra_registration.py'
python3 -m unittest discover -s web/tests -p 'test_camera*.py'
node --test web/tests/astra-registration.test.mjs web/tests/state.test.mjs
```

Tests use synthetic parameters only: ABI/non-finite rejection, identity/profile matching, proper transforms, identity projection, transformed Z, negative depth, occlusion, factory mode readback, raw fallback, and UI distinctions. They do not replace physical calibration.

Sources:
- [Pinned Orbbec parameter interface and camera-info conversion](https://github.com/orbbec/ros_astra_camera/blob/210f530144314da7c70072215bacba12130d9b20/src/ob_camera_info.cpp)
- [Pinned 120-byte OBCameraParams declaration](https://github.com/orbbec/ros_astra_camera/blob/210f530144314da7c70072215bacba12130d9b20/include/openni2/OniCTypes.h)
- [OpenCV camera and stereo calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [OpenCV depth registration](https://docs.opencv.org/4.x/d2/d3a/group__rgbd.html)

## Stationary RGB/IR capture through the preview owner

`camera_preview.py` now polls `web/hardware/runtime/astra-calibration/request.json`. A request must contain a new 32-character lowercase hex `token` and the current capture `session` from `camera-depth-control.json`. Stale-session, malformed and repeated requests are ignored. A capture is rejected while Astra depth is requested or active. It runs synchronously in the sole Astra worker, reuses its UVC RGB handle, opens only the IR stream, and closes/unloads OpenNI before resuming color preview. It never opens a second RGB pipeline or resets USB.

`result.json` reports `captured` or `error`. Successful frames are lossless `color.png` and `infrared.png` under the token directory; IR is intensity, **not distance**. Host receipt times are logged separately and are not exposure synchronization. Both board and camera must stay stationary for a pair to be suitable for calibration. All samples and request/results are ignored runtime data.

A camera-only reload with `python3 web/src/camera_preview.py --skip-camera d435` keeps the faulty D435 unopened and retains D405/Astra color. The original no-argument launcher behavior is unchanged. Three robot-driver PIDs were unchanged during this camera-only reload. The first real IR pair was captured successfully, but projected dots obscure the board: RGB detected 20 markers/15 ChArUco corners, IR detected none. This pair is diagnostic evidence, not a calibration solution.
