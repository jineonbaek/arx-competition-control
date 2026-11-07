# Astra SDK/parameter audit

Scope: camera-only compatibility diagnosis. No hand calibration, robot driver changes, firmware flashing, emitter writes, USB reset or CAN access.

## Baseline observed 2026-11-07/08 (Hong Kong)

- Actual loaded core: `vision/vendor-openni/libOpenNI2.so`, version **2.3.0.88**, SHA256 `29e9cafea8fec360edf50f88e2afc5b4b793664d28b6d91e144095893f0efe17`.
- Sensor plugin SHA256: `116aade45a425af1dbbf50c1afec227c415594b78a187bf9bd2c2254c56fee3e`; reported driver version **5.2.0.0**.
- Device: USB `2bc5:0403`, firmware **5.8.22**, serial **17122520757**.
- Ubuntu also has stock OpenNI **2.2.0.33**, but the preview process maps the private **2.3.0.88** core. `ldd` finds its dependencies. This rules out accidental loading of the system core in the observed process, not every compatibility problem.
- Existing Python uses the Orbbec-specific **1028-byte** device-info structure and **120-byte** `OBCameraParams`. Hashes match the pinned ROS repository downloads.

## Native reproduction

`vision/tools/astra_sdk_probe.cpp` uses the matching official C++ headers and C API backend, without Python. A canary follows the calibration buffer; checked size is 120 bytes. Tests show the same **all-FF** payload with status OK at device-open, after configuring 640x480@30 1-mm depth, and after starting depth. All buffer canaries remain intact. This excludes a Python-only parsing error for the observed payload.

Real native depth produced 230656 nonzero pixels out of 307200 in one frame. This confirms transport/output, not distance accuracy. SDK depth-to-world conversion also returned finite values for deliberately **synthetic** Z=1000 mm inputs; those are coordinate-conversion checks, not measured 3D points or recovered RGB-D calibration. The results are consistent with its FOV projection and do not prove that factory RGB intrinsics/extrinsics were recovered.

Raw logs/downloads/binaries are under ignored `references/astra-sdk-audit/`. The official **2.3.0.86 beta6** archive is isolated there, SHA256 `2240d645f3127bfcefbd2909be35bb35a446661e6a76d97dead414a4c5d684ad`; it has not replaced the live SDK. Cross-version device results will be recorded after testing.

## Reproduce safely

Compile against the matching Orbbec include/library directories, using an absolute source path. Do not compile against Ubuntu's stock `OniDeviceInfo` ABI. The probe itself refuses to initialize OpenNI while the existing camera capture lock is held. Stop only camera preview gracefully, wait for exit, run one SDK probe at a time, then restore preview with D435 skipped. Never stop robot drivers for this test.

```bash
# SDK_INCLUDE / SDK_LIBRARY refer to matching privately unpacked vendor files.
g++ -std=c++17 -Wall -Wextra -Werror -I"$SDK_INCLUDE"   "$PWD/vision/tools/astra_sdk_probe.cpp" -L"$SDK_LIBRARY"   -Wl,-rpath,"$SDK_LIBRARY" -lOpenNI2 -o /tmp/astra_sdk_probe
/tmp/astra_sdk_probe --self-test  # ABI/canary checks; no SDK initialization
/tmp/astra_sdk_probe             # Opens Astra, configures but does not start depth
/tmp/astra_sdk_probe --stream    # Brief depth stream, then closes it
```

Validation: warnings-as-errors native build, ABI/canary/all-FF self-check, live-owner lock refusal (exit 3 before SDK initialization), and native real-device runs. During the camera-only test the three robot driver PIDs/start times remained unchanged. No firmware/calibration was written to the camera.

Sources: [pinned official headers](https://github.com/orbbec/ros_astra_camera/tree/210f530144314da7c70072215bacba12130d9b20/include/openni2), [official OpenNI release](https://github.com/orbbec/OpenNI_SDK/releases/tag/v2.3.0.86-beat6).
