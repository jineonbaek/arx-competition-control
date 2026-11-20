# Reusable Taught Placement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship and immediately test a reusable one-click 3 rad battery placement flow that replays the successful taught topology, uses current D435/D405 offsets at the mouth, fully inserts, releases, reverses out, returns home, preserves overlays, and records every stage duration.

**Architecture:** Add a focused taught-route/timeline module consumed by the existing `FastVisualLevel` controller. Keep the current running motor drivers untouched; deploy only Python/web/config assets into the live web observer tree. The automation records every actually reached joint reference so withdrawal is the exact reverse of the current run, while the page consumes the existing overlay artifacts with freshness fallback.

**Tech Stack:** Python 3, NumPy/OpenCV, existing controller/DockingMotion APIs, `unittest`, vanilla HTML/CSS/JavaScript.

**Spec:** `docs/superpowers/specs/2026-11-16-reusable-taught-placement-design.md`

## Global Constraints

- Platform target is 3.0 rad with a ready band of 3.0 ± 0.35 rad; no platform compensation during right-arm placement.
- No manual continue/release gate after start.
- Preserve emergency/cancel, feedback freshness, ownership, clamp-before-release, joint-corridor and arm/base/left-arm hard stops.
- Do not move the chassis, modify/restart motor drivers, delete calibrations, overlays or historical reports.
- Release only after measured full-depth completion; return by reversing this run's reached references.
- Existing red-rim and battery-outline overlays remain visible, with stale-overlay fallback to raw camera images.
- Every phase persists planned and actual timing, including failure phases.

## Review Focus

- D405 is temporarily occluded or stale: bounded retries occur, then the battery stays clamped and the arm returns to the safe observation pose.
- Release feedback does not reach open: withdrawal must not begin and the arm must hold.
- A run stops halfway through insertion: only reached references are reversed; unexecuted taught points are never used.
- Overlay metadata is stale or refers to a different image: raw live image is shown and stale geometry is not displayed.
- Wall-clock time changes during a run: `duration_s` still uses the monotonic clock and remains nonnegative.

---

### Task 1: Taught Route, 3 rad Defaults, and Timeline

**Files:**
- Create: `config/taught-placement.json`
- Create: `agent/taught_placement.py`
- Create: `agent/tests/test_taught_placement.py`
- Modify: `agent/held_placement_geometry.py`
- Modify: `agent/tests/test_held_placement_geometry.py`
- Modify: `config/battery-execution.json`

**Interfaces:**
- Produces: `PhaseTimeline`, `TaughtRoute`, `load_taught_route(path)`, `reverse_reached_route(records)`, `prepared_height(height)`.
- Consumes: successful reports `full-insert-20261116-1620.json` and `full-insert-continue-20261116-1621.json` only as provenance/config input, never at runtime.

- [ ] Write failing tests for the 3.0 ± 0.35 rad predicate, versioned route values, reached-only reversal, nonnegative monotonic phase durations, failure persistence, and total-run duration.
- [ ] Run `python -m unittest -v agent.tests.test_taught_placement agent.tests.test_held_placement_geometry` and verify the new assertions fail for missing behavior.
- [ ] Implement the minimal route/timeline module and 3 rad defaults.
- [ ] Re-run the focused tests and the full `agent.tests` suite.

### Task 2: Full Automatic Execution

**Files:**
- Create: `web/src/taught_placement_live.py`
- Create: `web/tests/test_taught_placement_live.py`
- Modify: `web/src/fast_visual_level.py`
- Modify: `web/tests/test_fast_visual_level.py`

**Interfaces:**
- Consumes: `TaughtRoute`, `PhaseTimeline`, existing D435 localization, D405 vertical alignment, controller ownership, `DockingMotion`, and current driver references.
- Produces: `FastVisualLevel.action({'op':'auto_full'})`, a complete receipt/timeline/executed-route directory, and final states `complete`, `held`, or `blocked`.

- [ ] Write failing tests proving `auto_full` is the only new browser operation, no release occurs before measured full depth, D405 correction is bounded to 5 mm per step/15 mm cumulative, reached references reverse on failure, release feedback gates withdrawal, and return targets the captured start pose.
- [ ] Run the focused tests and verify RED failures come from missing APIs/behavior.
- [ ] Implement planning/execution in small measured segments, automatic D405 retries, full-depth release, reverse withdrawal, safe observation fallback, and phase/timeline persistence.
- [ ] Run focused tests, all web unit tests, and compile all changed Python files.

### Task 3: Overlay and Timing UI

**Files:**
- Modify: `web/public/visual-docking.html`
- Modify: `web/public/visual-docking.js`
- Modify: `web/public/visual-docking.css`
- Create: `web/tests/test_taught_placement_ui.py`
- Modify: `experiments/live-overlays-20261116/worker.py` only if metadata lacks a source-frame timestamp.

**Interfaces:**
- Consumes: `/api/fast-level` snapshot fields `timeline`, `phase_started_at`, `phase_duration_s`, `total_duration_s`, `planned_duration_s`; `/camera-overlay/{d435,d405}.json` and raw `/camera-live/*.json`.
- Produces: one-click “完整放置并自动返回”, live phase and total timers, depth/vertical correction metrics, fresh overlay selection with raw fallback.

- [ ] Write failing UI contract tests for the new operation, timing fields, preserved red-rim/battery-outline labels, no confirm-release button, and stale-overlay fallback.
- [ ] Run the UI tests and verify RED.
- [ ] Implement the HTML/JS/CSS changes without removing calibration links/assets.
- [ ] Run UI tests and the complete web test suite.

### Task 4: Deploy and Immediate Physical Validation

**Files:**
- Create/update: `scripts/deploy-taught-placement.py`
- Create: `reports/taught-placement-deployment.json`
- Runtime output: `reports/automatic-placement-*/`

**Interfaces:**
- Consumes: tested worktree assets and live web observer process identity.
- Produces: atomic live-tree deployment with backup, unchanged motor-driver identities, restarted web observer, one real `auto_full` run, and final D435 evidence.

- [ ] Write a deployment preflight that snapshots live hashes/process identities, rejects active motion, backs up only replaced assets, and compiles/tests worktree files.
- [ ] Run the preflight; verify right/left motor driver PIDs and start times are recorded.
- [ ] Atomically copy changed Python/config/web assets to `/home/arx/arx-competition-control`, stop/restart only the web observer, and verify fresh feedback plus unchanged motor-driver identities.
- [ ] Verify the battery is clamped, platform is near 3 rad, D435/D405 are fresh, and no controller owns motion.
- [ ] Start `{'op':'auto_full'}` through the live HTTP endpoint, monitor until terminal state, and issue arm/lift stop on any failure.
- [ ] Verify final D435 image, open gripper, returned start pose, released control, complete timeline, and per-stage durations; report any hard-stop reason instead of claiming success.

## Self-Review

- Spec coverage: route replay, fresh offsets, D405 trim, full insertion, automatic release/return, overlays and timing all map to Tasks 1–3; deployment and physical evidence map to Task 4.
- Interfaces: Task 2 depends only on Task 1 APIs; Task 3 depends on Task 2 snapshot fields; Task 4 consumes all earlier artifacts.
- Safety: no human confirmation gate is introduced; hard stops remain explicit and tested.
- Scope: chassis and motor drivers remain out of scope; existing calibration/overlay assets are preserved.
