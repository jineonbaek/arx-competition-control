# Faster lift with smooth reference motion

The operator requested further speed and smooth movement after activating the
1.0 SDK rad/s fixed ramp. Accuracy compensation remains deferred.

## Selected approach

Use a local, single-axis jerk-limited trajectory between SDK references, with
maximum velocity **2.0 rad/s**, acceleration **4.0 rad/s²**, and jerk
**16.0 rad/s³**. These are reference constraints and engineering defaults,
not manufacturer ratings or guarantees about measured motor motion. Short
travel need not reach cruise speed; long travel is faster than the old ramp.

A fixed faster ramp would keep abrupt velocity transitions. A custom seven-phase
solver would duplicate difficult reversal/numerical cases. Use pinned Ruckig
Community v0.15.3 (C++17, MIT), entirely offline, with no cloud client or Python
module. This is an implementation choice under the user's instruction to proceed
without routine approval questions. Actual activation still needs a fresh
supported-bottom handoff because restarting releases motor holding force.

## Contracts

- Keep the existing web/ROS target parameter, 0..18 SDK rad limits, exclusive
  CAN5 owner, calibration check, base SDK protection, gains, torque and motor mode.
- Keep homing at 0.5 rad/s and explicit lift.stop as immediate cancellation and
  hold of the current SDK reference. Normal requested motion has smooth starts,
  stops and mid-motion retargets; safety stop is not delayed for a smooth curve.
- Start from the SDK reference, not biased measured height. No endpoint offset
  correction or position-accuracy gate is introduced.
- Replan a changed target from current planned position/velocity/acceleration.
  Identical repeated targets must not restart the trajectory. Validate the whole
  calculated position extrema against 0..18 before execution.
- Advance from steady-clock elapsed time. Adapt the SDK's per-tick reference cap
  to elapsed time so its old 400 Hz increment does not distort the curve.
- SDK reference divergence, invalid state or a moving-loop gap above 50 ms
  cancels the goal and holds the last valid SDK reference. Do not auto-resume an
  interrupted motion without a new accepted request.
- Use a post-set parameter callback to enqueue only committed target requests;
  calibration loss and explicit stop cancel pending work.
- Publish reference motion status at 20 Hz: requested/reference/SDK positions,
  reference velocity, acceleration, jerk, limits, active/fault and instance ID.
  This permits independent comparison with actual motor feedback.

## Integration, restoration and validation

Add a pure LiftMotion wrapper, a native test executable and a static, pinned
Ruckig source copy under web/hardware/vendor, preserving the MIT license and a
file-hash manifest. No SDK source, Python runtime, database, UI or cloud changes.
Build in the independent worktree and deploy only the native body executable via
the existing versioned rollback/atomic switch procedure.

Test forward/reverse/short/boundary moves, analytic limits, C2 retargets, repeated
requests, explicit stop, calibration loss, reference divergence and irregular
tick durations. Run all pure native checks and compile the real driver without
executing it. Review the final patch before a real supported-bottom handoff.
Then capture both planned motion status and actual motor feedback during a
bounded onsite-supervised trial. Measured smoothness remains unproven until that
trial and onsite observation; reference smoothness cannot remove mechanical
friction by itself.
