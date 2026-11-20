# Visual docking implementation plan

> **For agentic workers:** use superpowers:executing-plans inline; one final fresh review.

**Goal:** Provide a reusable observe-plan-execute-reobserve docking loop and current-image plan without restarting loaded hardware.

**Architecture:** Pure aperture-fit and D405 monitoring modules feed a transport-independent coordinator. Existing cabinet planner/IK/loaded sweep provide the live read-only adapter; execution adapters receive complete typed plans and fresh feedback, never individual joint jog instructions.

**Tech Stack:** Existing Python, NumPy, OpenCV, timed trajectory controller.

**Spec:** `docs/superpowers/specs/2026-11-16-visual-docking.md`

## Global Constraints

- Never release gripper, restart loaded drivers, re-zero, change gains/torque, replay uncertain motion or disable emergency/freshness/collision limits.
- Preserve original engineering projects and unrelated dirty files; do not push.
- Current implementation pass is offline and read-only live verification; physical execution remains unproven.

## Review Focus

- Full battery length amplifies tilt; a tip-only fit is insufficient.
- Old frame or missing D405 view cannot qualify insertion.
- Uncertain insertion must not trigger a blind reverse plan or automatic retry.
- Cancellation must work independently of new camera success.
- Base/lift fallback cannot invent distance feedback or use blind timed motion.

### Task 1: Geometry and relative grip evidence

Files: `agent/docking_fit.py`, `agent/docking_grip.py`, `agent/tests/test_visual_docking.py`.

- [ ] Write RED tests for fitted and tilted envelopes, narrow aperture/uncertainty, stale frames, temporal grip displacement and no silent rebaseline.
- [ ] Implement `evaluate_fit(tool,window,slot,battery,contact,uncertainty_mm=0,margin_mm=.5)` and `GripTracker.bind/update(rgb,stamp,raw,now)`.
- [ ] Run geometry/tracker tests GREEN.

### Task 2: Reusable coordinator and real-image planner

Files: `agent/visual_docking.py`, `scripts/preview-visual-docking.py`, documentation/report.

- [ ] Write RED tests: one-run arm path, base/lift strategy, correction budget/progress, cancellation, stale/uncertain replies, hold-only terminal state and no release method.
- [ ] Implement coordinator backend protocol `observe`, `plan`, `execute`, `hold`, `cancelled` with durable phase records. Each plan identifies observation and whole checked trajectory; each execution receipt must confirm measured completion and fixed-Z for insert.
- [ ] Implement read-only SSH preview using existing cabinet planner, current D435 and D405, full sweep and fit report; never construct actuator transport.
- [ ] Run new and affected existing tests; final fresh review, one fix pass; verify live driver identities/epochs unchanged.
- [ ] Deliver structure, source and exact evidence/limitations. No runtime deployment or physical action claim.
