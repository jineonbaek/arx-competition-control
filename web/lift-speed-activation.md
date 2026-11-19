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

## Prepared deployment and rollback

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
rollback, unchanged homing, invalid rates and existing limits are covered. No higher-speed movement or restart
has yet been performed. Activation and measured speed results will be recorded
after onsite handoff and the authorized trial.

Runtime binaries, backups and recordings stay outside Git; restore them from
the selected source using the existing ROS 2 Jazzy/SDK build instructions.
There is no cloud component or new dependency.
