# Uninterrupted 160 mm Battery Placement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add one uninterrupted right-arm operation that performs D435 pre-adjustment or fixed-pose fallback, inserts 160 mm, releases, reverses the reached route, and returns.

**Architecture:** Add a distinct `auto_insert_160` mode to the existing `FastVisualLevel` owner so legacy modes stay unchanged. The new mode treats D435 as optional before engagement, selects exactly one entry source, then locks the insertion axis and uses the existing native trajectory, gripper, route reversal, and timing machinery.

**Tech Stack:** Python 3, NumPy/SciPy kinematics, existing ROS 2 adapter and native trajectory driver, vanilla browser JavaScript, unittest/pytest.

**Spec:** `docs/superpowers/specs/2026-11-19-uninterrupted-160mm-placement-design.md`

## Global Constraints

- Right arm and right gripper only; never command left arm, lift, chassis, or waist/head.
- D405 is not used.
- Full insertion depth is exactly 160.0 mm.
- Use repository motion limits without the legacy 8 mm/s and 4 mm/s2 insertion cap.
- No manual gate between entry selection, insertion, release, withdrawal, and return.
- Camera fallback is allowed only before insertion begins.
- No retry after partial insertion.
- Preserve all unrelated dirty-worktree changes.

## Review Focus

- D435 fails after making an outside correction: fixed fallback must still remain outside and reset the reached route consistently.
- Fixed waypoint file is missing, malformed, stale, or incompatible with current lift: reject before insertion.
- The start pose already differs materially from the recorded fixed pose: use a checked native trajectory, not an asynchronous joint jump.
- Release feedback never reaches open target: do not begin withdrawal and report the release phase failure.
- Cancellation or driver replacement during insertion: stop the native trajectory and retain an explicit partial-insertion failure receipt.

---

### Task 1: Route policy and entry-source selection

**Files:**
- Modify: `config/taught-placement.json`
- Modify: `agent/taught_placement.py`
- Test: `web/tests/test_taught_placement.py`

**Interfaces:**
- Produces: `TaughtRoute.full_depth_mm == 160.0`, `fixed_entry_waypoint: Path`, and validated fixed-entry data loader.

- [ ] Write failing tests asserting 160.0 mm depth, the exact fixed waypoint path, six finite right-arm joints, finite base wrist data, gripper target 0, and lift compatibility tolerance.
- [ ] Run the focused test and confirm it fails because the new route fields and loader do not exist.
- [ ] Extend the route schema and loader minimally; reject missing keys, non-finite values, wrong arm, wrong joint count, or lift mismatch over 0.10 rad.
- [ ] Run the focused test and confirm it passes.

### Task 2: Uninterrupted hybrid placement state machine

**Files:**
- Modify: `web/src/fast_visual_level.py`
- Test: `web/tests/test_fast_visual_level.py`

**Interfaces:**
- Consumes: validated taught route and fixed-entry waypoint from Task 1.
- Produces: accepted operation `{ "op": "auto_insert_160" }`, `entry_source` (`d435` or `fixed`), and one continuous phase timeline.

- [ ] Write failing tests for fresh-D435 selection, stale-D435 fixed fallback, no D405 access, no lift/chassis/left-arm command, and no fallback after `full_insert` starts.
- [ ] Run the focused tests and verify the expected failures.
- [ ] Add `auto_insert_160` as a separate mode; make D435 readiness optional only for this mode and suppress D405 monitoring.
- [ ] Implement fixed-entry motion through the existing checked native trajectory path, retain the closed gripper target, derive the tool +X entrance axis from the measured arrival pose, and record `entry_source=fixed`.
- [ ] Reuse D435 outer alignment when fresh; if it raises before engagement, move to the fixed entry and record the fallback reason.
- [ ] Lock the selected axis, execute exactly 160.0 mm, release, reverse `reached_route`, and require return to the captured start joints.
- [ ] Remove only the new mode's 8/4 insertion cap so `motion_limits()` supplies TCP 60 mm/s and 120 mm/s2 while the driver keeps its joint limits.
- [ ] Run the focused tests and confirm they pass.

### Task 3: Timing, receipts, and partial-insertion failure behavior

**Files:**
- Modify: `web/src/fast_visual_level.py`
- Test: `web/tests/test_fast_visual_level.py`

**Interfaces:**
- Produces: receipt fields `entry_source`, `fallback_reason`, `depth_mm`, `released`, `timeline`, `total_duration_s`, and terminal phase/status.

- [ ] Write failing tests proving every phase records planned, actual, and cumulative time; release precedes withdrawal; failure after insertion starts never invokes fixed fallback or a second insertion.
- [ ] Run the focused tests and verify failures.
- [ ] Add the minimal receipt updates and explicit partial-insertion failure state.
- [ ] Run focused tests and confirm they pass.

### Task 4: One-button web integration

**Files:**
- Modify: `web/public/visual-docking.js`
- Modify: `web/public/visual-docking.html`
- Test: `web/tests/visual-docking.test.mjs`

**Interfaces:**
- Consumes: `/api/fast-level` operation `auto_insert_160` and its receipt.
- Produces: one button that starts the full sequence and displays selected entry source, fallback reason, 160 mm depth, release, return, and phase timing.

- [ ] Write a failing browser test asserting the button posts `{op:"auto_insert_160"}` once and contains no intermediate continue/confirm action.
- [ ] Run the focused browser test and verify it fails because the button still posts `d435_only`.
- [ ] Change the button action and user-facing phase/status text without altering cancel behavior or overlay refresh.
- [ ] Run the focused browser test and confirm it passes.

### Task 5: Verification and deployment

**Files:**
- Modify only if a verification failure identifies a scoped defect.

**Interfaces:**
- Produces: tested source deployed to the existing web process; no physical placement run.

- [ ] Run focused Python tests for taught placement and fast visual placement.
- [ ] Run focused Node/browser tests for the visual-docking page.
- [ ] Run the complete web Python and Node test suites; report every pre-existing failure separately.
- [ ] Run Python syntax compilation and JavaScript syntax checks.
- [ ] Restart only the web service if required; do not restart arm/lift drivers.
- [ ] Verify `/api/fast-level` is reachable, the visual-docking button posts the new operation, ROS remains connected, and right-arm feedback is fresh.
- [ ] Inspect the final diff to ensure only scoped files plus the spec/plan changed; do not commit or push unless separately requested.
