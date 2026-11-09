# Three-camera depth acquisition (ROG)

The existing camera_preview.py is the only camera owner. All cameras start COLOR ONLY. Depth defaults OFF on every capture-process start. The obsolete YOLOE service has been removed. It never imports ROS, robot SDKs, or CAN. Keep its lock; do not open another camera pipeline beside it.

| Camera | Color / depth acquisition | Export | Color registration |
| --- | --- | --- | --- |
| D435 | RealSense SDK, 640x480 at 15 FPS | up to 5 frames/s | SDK depth-to-color alignment, factory intrinsics |
| D405 | Same RealSense worker, 640x480 at 15 FPS | up to 5 frames/s | SDK depth-to-color alignment, factory intrinsics |
| Astra Pro | Existing UVC color + OpenNI native depth 640x480 at 30 FPS | up to 5 frames/s | **Unregistered** separate streams; no hardware synchronization claimed |

Color previews stay available. Recognition is disabled. Each camera card has an independent depth control. Camera cards report depth state, valid-pixel fraction, and alignment. Color remaining online cannot hide a missing or stale depth stream in the one-click self-check.

## Data contract

Use `http://127.0.0.1:9983/camera-live/{d435,d405,astra}.json`. Each online depth object includes:

- `image_file`: immutable 16-bit single-channel PNG; load with `cv2.IMREAD_UNCHANGED`.
- `preview_file`: immutable false-color JPEG for viewing, **not** measurement data.
- `scale_m_per_unit`: multiply the stored integer by this to obtain metres. Zero is invalid, not a zero-distance obstacle.
- `width`, `height`, `intrinsics`, `aligned_to_color`, frame number, sensor timestamp, and `captured_at`.
- `valid_fraction`: nonzero pixels / total pixels. An online stream can have invalid pixels or no measurable scene; do not replace missing values with invented depths.
- `preview_range_m`: fixed display scale (D405 0-0.7 m, others 0-4 m). This is a visualization range, not a calibrated hardware range.

The JSON and its filenames refer to one capture publication. Read the JSON first, then the referenced PNG/JPEG; do not substitute the mutable latest color image. Only the latest 35 depth PNGs and previews per camera are retained; old-session depth files are removed on startup after 60 seconds. These are temporary live frames, not a recording archive.

RealSense depth timestamps and color/depth gaps are retained. Hardware-clock timestamps use host receipt for wall-clock freshness and explicitly say `host_receipt_only`; no cross-camera synchronization is claimed.

Astra depth has its own device timestamp and host receipt time. `pairing=independent_unsynchronized` and `aligned_to_color=false`: never use color pixel (u,v) to directly index this depth map. Its focal lengths are derived from SDK depth FOV and its principal point assumes image center (`SDK_FOV_derived_center_assumed`, unknown distortion), **not** a measured or complete factory calibration. RGB-depth registration and hand-eye calibration are still separate work before robot-frame grasp execution.

## Dependencies and Astra permissions

Stop the camera preview owner before installing dependencies: setup holds the existing capture lock and refuses to overwrite SDK libraries that are in use. System Python 3.12, NumPy and OpenCV are already installed. Camera dependencies live under ignored `vision/vendor-python/` and `vision/vendor-openni/`; robot Python environments are unchanged.

```bash
cd /home/arx/arx-competition-control
PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple python3 vision/scripts/setup-depth.py
sudo install -m 644 config/99-arx-astra-depth.rules /etc/udev/rules.d/99-arx-astra-depth.rules
sudo udevadm control --reload-rules
```

The rule grants group `plugdev` access only to Astra depth USB ID `2bc5:0403`; user `arx` is already a member. For a connected device apply a udev **change event to that device only**, or replug only that camera when safe. Do not reset a USB controller or trigger all USB devices: robot CAN adapters share USB infrastructure. A system rule copy is outside Git; rollback requires removing `/etc/udev/rules.d/99-arx-astra-depth.rules` and reloading rules. Git revert alone does not remove that installed copy or revoke an already-open USB handle.

The setup pins the official Orbbec ROS repository at `210f530144314da7c70072215bacba12130d9b20`, downloads only six Linux x64 runtime files (about 2.6 MB), and checks SHA256. The stock `openni==2.3.0` Python binding omits Orbbec's `serialNumber[256]` field in `OniDeviceInfo`. Setup adds it deterministically; runtime rejects the unsafe structure before SDK initialization. Without it the SDK writes beyond the Python buffer and may crash. This is a camera-only compatibility fix, not a driver or firmware change. Reinstall through this setup script so the patch is reapplied.

## D405-only recovery mode

To enable only D405 RGB-D, without touching D435/Astra or starting the detector:

```bash
python3 web/src/camera_preview.py --camera d405 --no-detector
```

This uses the same exclusive capture lock and existing HTTP camera metadata. Unselected cameras are reported offline; robot processes stay untouched. The desktop launcher starts all COLOR previews with depth OFF. This mode does not claim the shared USB fault is fixed.

## Start / verify

```bash
python3 scripts/start-console.py --cameras-only
python3 -m unittest discover -s web/tests -p 'test_camera*.py'
```

`--cameras-only` reuses a running camera owner; it does not reload changed Python. To apply capture changes, terminate only the confirmed camera-preview PID gracefully, wait for it to exit, then run the command above. This briefly interrupts camera previews/detection. It never requires a robot-driver restart, lift homing, motion commands, or USB reset.

Check all three JSON depth objects are online, timestamps and frame numbers advance, the referenced PNG decodes as uint16 480x640, and the color and detector outputs remain fresh. Do not interpret this stream verification as a physical distance accuracy or grasping test.

Sources: [RealSense alignment example](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py), [Orbbec OpenNI SDK](https://www.orbbec.com/developers/openni-sdk/), [pinned Orbbec runtime](https://github.com/orbbec/ros_astra_camera/tree/210f530144314da7c70072215bacba12130d9b20/include/openni2_redist/x64), [OpenCV Astra tutorial](https://docs.opencv.org/4.x/d0/db6/tutorial_orbbec_astra_openni.html).

## On-demand depth controls (2026-11-07)

Each 9983 camera card now has an independent depth button. Requests require the existing session cookie and same Origin. The HTTP response only acknowledges a request; the button waits for fresh capture metadata before confirming success. Offline color cameras cannot enable depth.

Disabling RealSense depth reconnects that camera with only its color stream enabled, without a depth transport stream. Astra stops its separate OpenNI depth stream while retaining UVC color. Switching RealSense mode may briefly pause that camera's color image. Failed depth starts are disabled automatically instead of continuously retrying depth.

All depth requests reset OFF on capture startup. A locked, atomically replaced request file lives in ignored web/hardware/runtime. Pausing or closing the page does not stop enabled depth: use the depth button.

The obsolete YOLOE weights, source and dedicated environment have been removed from this workspace. The preview uses raw camera frames; recognition runs only through the separate on-demand vision entry. Intentional depth OFF is healthy in camera self-checks.

On-demand depth reduces unnecessary USB traffic. It does not prove the cause of the earlier shared-USB fault has been fixed.

## Astra calibration validation

The Astra source now validates factory parameters, supports a measured calibration file bound to serial/profile, and keeps raw depth when registration cannot be established. Calibration availability and physical alignment validation are reported separately. See [Astra calibration evidence and procedure](ASTRA_CALIBRATION.md). No measured file has been installed for this camera; physical target capture and accuracy verification are still required. Existing capture processes need a camera-only reload to load source changes.
