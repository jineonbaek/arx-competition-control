# Held Battery and Local Docking Offline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Connect real RGB-based D405 held-object observations to phase-specific local cabinet search/docking decisions, verified exclusively offline.

**Architecture:** File-only reference/feature tracking feeds a temporal held-state monitor. Typed simulation-scoped evidence feeds a deterministic gatekeeper, bounded docking proposal planner, persistent run state and simulated stop/takeover arbiter. Replay integrates saved D435/D405 frames and produces a standalone report; no hardware transport or runtime service switch.

**Tech Stack:** Existing Python, NumPy, OpenCV, unittest; existing cabinet RGB/CAD localization and geometry. No new production dependency.

**Spec:** `docs/superpowers/specs/2026-11-15-local-autonomous-cabinet-design.md` and `docs/d405-held-battery-design.md`.

## Global Constraints

- No hardware/device/ROS control, new camera capture, driver start, running-service switch, push or auto-commit.
- Preserve the independent worktree and pre-existing changes; local export mirrors `/home/arx/arx-place-integration`.
- Every report says offline/simulation and executable=false. Simulation evidence cannot enable real execution.
- No point-cloud assumptions; D435 primary target, D405 held-object evidence; obscured dimensions remain unknown.
- Reuse TCP/handeye identity without promoting nominal/candidate calibration to verified accuracy.
- No automatic tightening, regrasp, torque change, emergency-stop release or retry of unknown physical outcomes.

## Review Focus

- Wrong but stable first observation must not become a valid grip baseline.
- Duplicate/future/stale frames and changed calibration must not refresh holding permission.
- Read-only inspection and stop remain available despite failed movement gates; takeover needs post-stop fresh rest evidence.
- Partial insertion and release ambiguity survive cancellation/restart and cannot loop into base/lift movement.
- Lost target, missing obstacle coverage or insertion precision cannot be bypassed by retries, alternative phases or synthetic fixtures presented as real.

### Task 1: D405 RGB observations and held-state monitor

**Files:** `agent/held_battery.py`, `agent/held_rgb.py`, corresponding `agent/tests/test_held_*.py`.
**Interfaces:** `ReferenceMatcher.analyze(rgb, frame_id, captured_at)` returns image-plane feature evidence with identity/observability. `HeldBatteryMonitor.update(observation, feedback, now)` returns presence, engagement, pose, stability and eligibility. Calibrated 3D/force remains unavailable unless independently supplied; RGB-only matches never fabricate it.

- [x] RED tests: correct partial ROI, empty reference, image-plane misgrasp/slip, wrist co-motion, occlusion, duplicates, stale frames, wrong identity and baseline replacement.
- [x] Implement conservative reference-based feature fitting with jaw anchors; untrusted/missing reference stays unknown.
- [x] GREEN tests and annotated synthetic/archived replay evidence. References are explicit offline datasets, not auto-approved first frames.

### Task 2: Gates, docking proposals and stop/takeover arbitration

**Files:** `agent/cabinet_gates.py`, `agent/cabinet_docking.py`, `agent/cabinet_simulator.py`, corresponding tests.
**Interfaces:** `evaluate(action, evidence, context, now)` produces required/blocking gates and simulation eligibility. `DockingPlanner.propose(...)` ranks bounded base/lift candidates, with injected kinematic/collision checks; unverified normal/stopping envelope stays blocked. `SimulationExecutor` owns an exclusive generation, accepts stop independently of movement gates, and transfers ownership only after fresh post-stop rest samples.

- [x] RED tests: G09 integration; search without target; diagnostic under faults; missing SI/stop/obstacle/precision evidence; budget and all normal/stopping-envelope checks.
- [x] RED tests: manual stop priority, unknown stop/rest, stale generation/TTL, duplicate segment, conflicting owners and borrowed parent token.
- [x] Implement typed simulation-only gate evidence, actual phase dependency table and cumulative budgets; run GREEN.

### Task 3: Persistent autonomous flow and recovery

**Files:** `agent/cabinet_autonomy.py`, integration tests and offline scenario fixtures.
**Interfaces:** `OfflineCabinetRun.step(...)` connects D435 tracking, held-state G09, gate decisions, docking choices and simulated receipts; state persists engagement, budgets, stop latch and inflight action. Pause/takeover/resume/cancel are generation-bound. No real Controller injected or imported.

- [x] RED tests: positive search→base→lift→outside flow with explicit synthetic evidence; D405 slip/unknown blocks the next segment; insertion precision stays independently gated.
- [x] RED tests: monotonic correction/oscillation budgets, post-command re-observation, partial insertion, independent retreat, unknown release and reboot; no automatic resume or repeat of release.
- [x] Implement and run GREEN, retaining all gates and unknown reasons in reports.

### Task 4: Offline replay, reviewable report and regression

**Files:** `scripts/replay_cabinet_autonomy.py`, `docs/held-docking-offline.md`, replay tests; case-to-test coverage report.
**Interfaces:** CLI consumes only saved NPZ/images plus explicit reference/scenario files, creates JSON/annotated HTML report; never captures or contacts hardware. Simulator scenarios are visibly separate from recorded observations.

- [x] RED CLI integration tests before implementation; run real RGB matcher and gatekeeper in the same replay path.
- [x] Exercise applicable A01–A58 requirements with assertions; report unsupported physical evidence as missing, not passed.
- [ ] Run full agent/web/browser regression on the independent host source, request one fresh review, fix consequential findings RED→GREEN.
- [ ] Guarded source-only sync with backups; leave online service unchanged and do not push.

Ruling: user approved the design and explicitly asked implementation/offline validation, so execute directly without another planning approval. Hardware gateway deployment and physical calibration certificates remain outside this offline authorization; the simulator cannot be substituted with real transport.

Ruling: 58-case traceability distinguishes implemented offline contracts from unimplemented real gateway, full retreat/joint executor, packing optimization and field datasets. Do not count these physical requirements as passed. Focused new suite87 passed; full host regression pending.
