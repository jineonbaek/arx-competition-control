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

Raw logs/downloads/binaries are under ignored `references/astra-sdk-audit/`. The official **2.3.0.86 beta6** archive is isolated there, SHA256 `2240d645f3127bfcefbd2909be35bb35a446661e6a76d97dead414a4c5d684ad`; it has not replaced the live SDK. Cross-version results follow below.

## Reproduce safely

Compile against the matching Orbbec include/library directories, using an absolute source path. Add `-DARX_OPENNI_LEGACY_60` only for the pinned 2019 2.3.0.60 headers/library; omit it for the tested modern 2.3.0.86/88 runtimes. Do not compile against Ubuntu's stock `OniDeviceInfo` ABI. The probe itself refuses to initialize OpenNI while the existing camera capture lock is held. Stop only camera preview gracefully, wait for exit, run one SDK probe at a time, then restore preview with D435 skipped. Never stop robot drivers for this test.

```bash
# SDK_INCLUDE / SDK_LIBRARY refer to matching privately unpacked vendor files.
g++ -std=c++17 -Wall -Wextra -Werror -I"$SDK_INCLUDE"   "$PWD/vision/tools/astra_sdk_probe.cpp" -L"$SDK_LIBRARY"   -Wl,-rpath,"$SDK_LIBRARY" -lOpenNI2 -o /tmp/astra_sdk_probe
/tmp/astra_sdk_probe --self-test  # ABI/canary checks; no SDK initialization
/tmp/astra_sdk_probe             # Opens Astra, configures but does not start depth
/tmp/astra_sdk_probe --stream    # Brief depth stream, then closes it
```

Validation: warnings-as-errors native build, ABI/canary/all-FF self-check, live-owner lock refusal (exit 3 before SDK initialization), and native real-device runs. During the camera-only test the three robot driver PIDs/start times remained unchanged. No firmware image or calibration was written to nonvolatile camera storage; optional stream tests apply normal transient camera settings.

Sources: [pinned official headers](https://github.com/orbbec/ros_astra_camera/tree/210f530144314da7c70072215bacba12130d9b20/include/openni2), [official OpenNI release](https://github.com/orbbec/OpenNI_SDK/releases/tag/v2.3.0.86-beat6).

## Completed SDK comparison

All SDKs were tested in private directories against their own vendor headers/libraries, one process at a time. None replaced the preview SDK.

| SDK core | Source | Device-info ABI | Full RGB-D parameter result | Native depth nonzero pixels, one frame |
| --- | --- | --- | --- | --- |
| 2.3.0.88 | Existing pinned ROS runtime | 1028 bytes | Five phase reads, all 120 bytes FF | 230656 / 307200 |
| 2.3.0.86 | Official OpenNI beta6 release | 1028 bytes | Five phase reads, all 120 bytes FF | 221296 / 307200 |
| 2.3.0.60 | Official ROS Astra commit `851c312c942a68b9431e51e798df28ce670ac89f` (2019) | 772 bytes | Five phase reads, all 120 bytes FF | 231581 / 307200 |

The nonzero-pixel counts came from different frames/times; they are not a benchmark of version accuracy or performance. No ruler/distance calibration was done in this audit.

For each SDK, reads were tested after opening the device, configuring 640x480@30 depth, and starting depth. The two native runs per SDK yield five checked reads (15 total). Every call reports status OK, 120 returned bytes and an intact canary. The SDK's native API therefore reproduces the payload seen in Python.

The 2019 driver also reports platform string **RD108C-009** and `Sensor params invalid(NaN value)` when requesting registration algorithm parameters. It reads other fixed depth-conversion values: ZPD=130, ZPPS=0.113967, LDDIS=7.5 and DCRCDIS=2.63 (vendor log names/units). Thus this is not a failure of all camera reads. These fixed values and the finite SDK converter do **not** recover the missing full RGB-D intrinsics/distortion/extrinsics, and do not establish physical accuracy.

### Conclusions and limits

- Ruled out for these measurements: accidental use of the Ubuntu OpenNI core; a Python-only parsing issue; the 1028-byte binding layout mismatch in the current deployment; registration parameters becoming available just by starting the stream; and a fix from switching among the three tested OpenNI versions.
- The older 772-byte ABI is real. The probe uses an explicit `ARX_OPENNI_LEGACY_60` compile flag only with the pinned 2.3.0.60 headers/runtime. Modern builds accept only the tested 2.3.0.86/88 runtime versions. Deliberately swapped ABI/runtime tests reject before SDK initialization. This prevents treating a successful compile as proof of runtime ABI compatibility.
- No valid full factory calibration has been recovered. The evidence does not distinguish an unsupported/absent calibration block for this old hardware/firmware from damaged or missing stored data, nor rule out every untested SDK/firmware compatibility issue. All-FF is not proof that flash was erased. No alternative structure was blindly passed to the same property, and no firmware was flashed.
- Native probe exit 0 means the diagnostic ran, not that calibration is valid. Inspect the returned bytes and values.

### Questions for manufacturer support

Device identity: **Astra Pro, USB 2bc5:0403, SN 17122520757, OpenNI FW 5.8.22, platform RD108C-009**, Ubuntu 24.04 x64. Three OpenNI versions and the native C++ reproducer return status OK but an all-FF 120-byte property-14 payload; native depth streams work.

1. Is full RGB-D calibration via property 14 expected to exist for this exact model/platform/serial, or is this an unsupported factory-data format/block?
2. Which supported SDK/tool/API and buffer layout should retrieve it for this firmware?
3. Can the original calibration for this serial be exported/provided, or is a manufacturer service procedure required?

Reproducer source is tracked; raw results and vendor packages are deliberately excluded from Git. No support message has been sent.

### Final checks

- Three warnings-as-errors native builds, three ABI/canary self-tests, three refusals while the live camera owner holds the lock, and two deliberate mismatched runtime/header refusals passed.
- Six real-device native runs completed; all 15 calibration payloads were checked as exactly 120 FF bytes with intact guards.
- Original preview SDK files remain unchanged. Astra and D405 color preview were restored with depth OFF; the already-faulty D435 remained skipped.
- Robot driver PID/start-time baselines remained `7360/27507`, `7502/27601`, `7650/27696` throughout. No robot driver, CAN setting, hand calibration or firmware update was performed.

Additional source: [official 2019 SDK commit](https://github.com/orbbec/ros_astra_camera/tree/851c312c942a68b9431e51e798df28ce670ac89f/include/openni2_redist/x64).
