# D435/D405 Closed-Loop Placement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace stale taught-cabinet replay with fresh D435 outer alignment, D405 vertical closure, verified full insertion, automatic release, reverse-path return, and per-stage timing.

**Architecture:** Add small pure helpers for visual convergence and phase timing, then make `FastVisualLevel.auto_full_place` orchestrate current-frame D435 iterations followed by D405-only vertical corrections. Preserve the existing controller, motion ownership, overlays, collision models, and configured demo left-slot target.

**Tech Stack:** Python, NumPy, OpenCV, existing ARX web controller, vanilla HTML/JavaScript.

**Spec:** `docs/superpowers/specs/2026-11-16-d435-d405-closed-loop-placement-design.md`

## Global Constraints

- Platform stays near 3 rad; no chassis, platform precision, or left-arm compensation.
- D405 has no fixed +10 mm fallback.
- Release is permitted only after measured full-depth arrival.
- Preserve D435/D405 red-rim and battery-outline overlays.
- Record planned, actual, and cumulative duration for every stage.

## Review Focus

- D435 frame where the battery or arm hides a rim edge must stop before movement.
- D435 outer-loop residual must use the configured demo left-slot target and current window, not a saved window.
- D405 unavailable evidence must stop without applying learned bias.
- Cancellation during full insertion must retain the clamp and stop the active arm trajectory.
- Reverse return must contain only waypoints actually reached in this run.

### Task 1: Pure convergence and timeline receipts

**Files:**
- Modify: `agent/taught_placement.py`
- Test: `agent/tests/test_taught_placement.py`

**Interfaces:**
- Produces: `outer_alignment_error(fit, window, slot) -> dict`
- Produces: `PhaseTimeline.plan(seconds)` and timeline rows with `planned_duration_s`, `duration_s`, `cumulative_duration_s`

- [ ] Write failing tests for current-window left-slot residuals and cumulative timing.
- [ ] Run the focused tests and confirm they fail for missing behavior.
- [ ] Implement the helpers without controller dependencies.
- [ ] Run the focused tests and confirm they pass.

### Task 2: D435 current-frame outer closed loop

**Files:**
- Modify: `web/src/fast_visual_level.py`
- Test: `web/tests/test_fast_visual_level.py`

**Interfaces:**
- Consumes: `outer_alignment_error`
- Produces: `d435_outer_align()` with current-frame observe/plan/move iterations until convergence or cancellation, without a correction-count or overall-loop timeout

- [ ] Write failing orchestration tests proving stale reference files are unused and non-convergence retains the clamp.
- [ ] Run the focused tests and confirm RED.
- [ ] Replace taught observation/static cabinet replay with fresh `pose()` plus `cabinet()` observations and bounded re-observation.
- [ ] Run focused tests and confirm GREEN.

### Task 3: D405-only vertical closure and verified insertion/release

**Files:**
- Modify: `web/src/fast_visual_level.py`
- Test: `web/tests/test_fast_visual_level.py`

**Interfaces:**
- Produces: `auto_d405_trim()` that returns only measured correction and raises when all attempts lack evidence
- Preserves: `execute_locked_axis()` and `release_and_return()` ordering

- [ ] Write failing tests for no learned-bias fallback and release-after-depth ordering.
- [ ] Run the focused tests and confirm RED.
- [ ] Remove fallback, bind the entrance axis to the latest D435 window, and persist reached waypoints.
- [ ] Run focused tests and confirm GREEN.

### Task 4: Web timing and overlays

**Files:**
- Modify: `web/public/visual-docking.html`
- Modify: `web/public/visual-docking.js`
- Modify: `web/public/visual-docking.css`
- Test: `web/tests/test_taught_placement_ui.py`

**Interfaces:**
- Consumes: API `timeline` rows
- Produces: phase table with planned, actual, cumulative duration while retaining `/camera-overlay/d435` and `/camera-overlay/d405`

- [ ] Write failing UI assertions for timing columns and retained overlay URLs.
- [ ] Run the UI test and confirm RED.
- [ ] Implement the table and live fields.
- [ ] Run the UI test and confirm GREEN.

### Task 5: Deploy and staged physical verification

**Files:**
- Deploy only the scoped Python/config/web assets after backup.

- [ ] Run focused tests, compile checks, and the full suite; report unrelated failures separately.
- [ ] Verify current hardware is stationary and clamped before restarting only the web service.
- [ ] Do not start a new physical run until the battery is restored to the task start pose.
- [ ] On the next run, monitor each phase receipt and verify full depth, open feedback, and return before reporting success.
