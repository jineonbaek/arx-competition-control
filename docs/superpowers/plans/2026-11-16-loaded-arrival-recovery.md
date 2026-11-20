# Loaded arrival recovery implementation plan

> **For agentic workers:** use superpowers:executing-plans inline; one final fresh review.

**Goal:** Fix completed-trajectory hold rebasing and prepare observable recovery without changing the live loaded driver.

**Architecture:** Pure tested selection of the stop reference in `arm_limits.hpp`, integrated at the driver stop boundary. Retain the existing gripper target helper, with validated optional startup restore and status telemetry.

**Tech Stack:** C++17, ROS 2 Jazzy, existing SDK.

**Spec:** `docs/superpowers/specs/2026-11-16-loaded-arrival-recovery.md`

## Global Constraints

- Never release gripper; do not loosen tolerances, modify gains, restart loaded hardware or replay failed movement.
- Independent source/build only; protected original engineering projects untouched; no push.

## Review Focus

- Running/faulted trajectories must not retain a future endpoint.
- Invalid/oversized/out-of-range/over-2-degree references fall back to measured.
- Explicit zero is a valid retained clamp target; invalid startup targets fail before SDK construction.
- Startup and mode changes cannot inherit stale issued references.
- Build/test is not evidence of physical tracking accuracy or insertion success.

### Task 1: Reference hold policy and observability

Files: `web/hardware/arm_limits.hpp`, `test_arm_limits.cpp`, `x5_controller_web.cpp`.

- [ ] Add failing regression assertions for completed bounded reference preservation and all fallback cases.
- [ ] Add pure policy helpers; integrate at stop before trajectory cancellation, preserve startup default and validate explicit startup clamp before SDK construction.
- [ ] Publish issued joint reference and commanded gripper target in existing status.
- [ ] Run all existing C++ safety tests; build the driver into a separate directory without executing it; final static review.
- [ ] Record evidence and request only the necessary mechanical-support state before live replacement. Do not commit or publish unrelated dirty changes.
