# Lift speed-first delivery — 2026-11-19

The operator requested speed before accuracy. The compensation experiment is
deferred; it has not been executed or connected to either runtime entry point.

## Behavior and scope

The native body driver now defaults to a lift reference ramp of **1.0 SDK rad/s**
instead of 0.5. The 400 Hz vendor reference increment is 0.0025 rad/tick.
Homing remains 0.5 rad/s. Gain 2, gravity feedforward -2, torque limit, MIT
motor mode and chassis control are unchanged. This doubles the reference rate;
the physical travel rate still needs measurement.

The read-only startup parameter is still `lift_target_ramp_rad_s`; explicit
0.5 remains available for rollback. Rate must be finite and within (0, 1.0].
Only the native body executable is to be activated. No Python/web precision
logic, calibration, database, UI or gripper service is changed.

## Deployment and rollback

Build from the independent candidate worktree and preserve the current active
executable by hash. Its expected original SHA-256 is
`f349c7bce9013415d3f2269c20ed4f1e374592b3dc086c5c59906306205f8614`.
Do not overwrite the original LIFT project or other tasks' source changes.

The existing launcher identifies the body driver by its original executable
argument path. Use a versioned executable under the existing ignored runtime
directory and an atomic symlink switch at the original executable path, retaining
the old binary in the deployment backup. The existing start-body command then
selects the new 1.0 default without changing startup arguments. Record selected
source commit, binary hash, old/new PID and read-back parameter. A later rebuild
from an older source checkout can restore the old default; restore the selected
candidate source before rebuilding.

Before switching, use the existing controller to lower at the original speed,
wait for stable fresh feedback, and obtain onsite confirmation that the platform
is physically at the lowest position and supported. Software-zero tolerance is
not proof of physical support. Restart briefly releases holding force and homes
the body controller; never restart a raised or unsupported platform.

After confirmation, stop only the verified body PID, check it exits without a
forced kill, switch the executable, and start exactly one replacement using the
existing CAN5 ownership lock and SDK. Keep both arm drivers and their state.
Verify calibration, one publisher, zero lift target and the 1.0 read-back rate.
Rollback also requires a supported low position; restore the old binary link
and existing 0.5 startup only after the same physical handoff checks.

## Validation and activation status

The default-rate test first failed at 0.5 and then passed at the new 1.0
default. Native driver rebuild and five pure C++ checks pass. Explicit 0.5
rollback, unchanged homing, invalid rates and existing limits are covered. The operator approved lowering with the old service, then explicitly confirmed
the mechanical bottom, stable support and a clear workspace. The driver was
switched and a low-position 0 → 2 → 0 trial completed.

### Activated result

- Source for the native executable: 5d0485033a53ad570ae46aa2b2d7172bdddd8463.
- New binary SHA-256:
  ce6963476659a072bc5e44e893745667c3886b3d97f23b039dac9554fc05613a.
- Old PID 5922 exited; replacement PID 1312881 is the single body owner.
- Startup parameter read-back: robot_type=0, fixed_body_height=0.0,
  lift_target_ramp_rad_s=1.0. Homing remains 0.5 by the unchanged setting.
- New calibration and low holding feedback verified against real motor frames;
  both arm-driver processes remained alive with their original identities.
- SDK references 0 → 2 → 0 completed without motor fault indications.
  The 10–90% measured-motion averages were about 0.863 rad/s upward and
  0.907 rad/s downward; sampled peak motor speed was about 1.659 rad/s.
  These are SDK feedback units, not calibrated millimetres per second.
- The final reference is 0.0. Measured endpoint offset remains; accuracy work is
  deliberately deferred. This test verifies speed operation over a short stroke,
  not full-range, loaded or absolute-position performance.

Before the shared-body handoff, existing navigation output identities were
stopped through the web interface and navigation action cancellation was
requested. No saved route definitions were deleted. Navigation was not
automatically resumed; start a new navigation task when appropriate.

The original executable is preserved as previous-lift_controller_web under
web/hardware/runtime/lift-speed-releases/ce6963476659/. The original executable
argument path now points to the versioned release, so the existing one-click
launcher still identifies and restarts exactly one body owner at the new default.
The private manifest records hashes, process identity, onsite confirmation and
read-back checks. Raw trial samples remain in the ignored candidate folder
.superpowers/lift-accuracy-speed/speed-1.0-trial-20261119/.

Runtime binaries, backups and recordings stay outside Git; restore them from
the selected source using the existing ROS 2 Jazzy/SDK build instructions.
There is no cloud component or new dependency.
