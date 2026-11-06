# Three-camera depth acquisition (ROG)

The existing `web/src/camera_preview.py` is the only camera owner. It never imports ROS, robot SDKs, or CAN. Keep its lock; do not open another camera pipeline beside it.

| Camera | Color / depth acquisition | Export | Color registration |
| --- | --- | --- | --- |
| D435 | RealSense SDK, 640x480 at 15 FPS | up to 5 frames/s | SDK depth-to-color alignment, factory intrinsics |
| D405 | Same RealSense worker, 640x480 at 15 FPS | up to 5 frames/s | SDK depth-to-color alignment, factory intrinsics |
| Astra Pro | Existing UVC color + OpenNI native depth 640x480 at 30 FPS | up to 5 frames/s | **Unregistered** separate streams; no hardware synchronization claimed |

Existing color previews and battery detections stay available. Camera cards report depth state, valid-pixel fraction, and alignment. Color remaining online cannot hide a missing or stale depth stream in the one-click self-check.

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

This uses the same exclusive capture lock and existing HTTP camera metadata. Unselected cameras are reported offline; robot processes stay untouched. The normal desktop launcher still defaults to all cameras: do not request camera recovery there during this isolated D405 check. This mode does not claim the shared USB fault is fixed.

## Start / verify

```bash
python3 scripts/start-console.py --cameras-only
python3 -m unittest discover -s web/tests -p 'test_camera*.py'
```

`--cameras-only` reuses a running camera owner; it does not reload changed Python. To apply capture changes, terminate only the confirmed camera-preview PID gracefully, wait for it to exit, then run the command above. This briefly interrupts camera previews/detection. It never requires a robot-driver restart, lift homing, motion commands, or USB reset.

Check all three JSON depth objects are online, timestamps and frame numbers advance, the referenced PNG decodes as uint16 480x640, and the color and detector outputs remain fresh. Do not interpret this stream verification as a physical distance accuracy or grasping test.

Sources: [RealSense alignment example](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py), [Orbbec OpenNI SDK](https://www.orbbec.com/developers/openni-sdk/), [pinned Orbbec runtime](https://github.com/orbbec/ros_astra_camera/tree/210f530144314da7c70072215bacba12130d9b20/include/openni2_redist/x64), [OpenCV Astra tutorial](https://docs.opencv.org/4.x/d0/db6/tutorial_orbbec_astra_openni.html).
