# Smooth lift speed implementation plan

> **For agentic workers:** Use superpowers:executing-plans for inline execution.

**Goal:** Raise cruise speed to 2.0 SDK rad/s with continuous reference velocity
and acceleration, preserving motor ownership and the existing stop contract.

**Spec:** ../specs/2026-11-19-lift-smooth-speed-design.md.

**Architecture:** Pinned offline Ruckig plus a pure stateful LiftMotion wrapper;
post-set ROS target events feed it, while the original SDK loop retains all
hardware writes and protection. Reference telemetry supports physical validation.

**Tech stack:** C++17, Ruckig v0.15.3, ROS 2 Jazzy, existing vendor SDK.

## Constraints and review focus

- Bounds: position 0..18, reference V=2, A=4, J=16; homing 0.5 unchanged.
- Target reversals retain current velocity/acceleration; duplicates do not replan.
- Stop and fault cancel queued motion; calibration recovery cannot replay it.
- Real elapsed-time sampling must survive SDK per-tick clipping without distortion.
- Software only until a supported-bottom handoff and clear supervised trial.

### Task 1: Pure reference planner

Files: vendor/ruckig with source/license lock; lift_motion.hpp;
test_lift_motion.cpp; CMakeLists.txt, slow_lift_config.hpp, test_body_limits.cpp.
Interface: LiftMotion.request(goal), advance(sdk_reference, dt, calibrated),
hold(reference), state/target/active/fault accessors and sdk_step_limit(dt).

- [ ] Write and observe failing native tests for limits, long/short/reverse moves,
  continuous retargets, duplicates, fault cancellation and SDK tick adaptation.
- [ ] Implement the offline wrapper and static dependency integration.
- [ ] Run the focused planner and all pure native checks.

### Task 2: Driver integration and deliverable

Files: lift_controller_web.cpp; feature/activation documentation.

- [ ] Feed accepted post-set target events, preserve immediate reference stop,
  shape targets before the existing SDK loop, and publish motion telemetry.
- [ ] Compile the native driver without executing it and review the patch.
- [ ] Update behavior/APIs/dependency restore/startup/testing/limitations docs;
  commit only reviewed source, tests and docs, not runtime assets.
- [ ] Prepare a versioned rollback release, then obtain fresh supported-bottom
  confirmation before activation. Record the actual supervised trial separately.
