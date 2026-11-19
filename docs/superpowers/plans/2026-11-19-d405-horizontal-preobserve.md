# D405 Horizontal Pre-observation Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fixed post-navigation pre-observation waypoint and a bounded D405 red-rim-to-battery-front-edge base-Y correction before the existing fixed-Z, true-`+X` 160 mm placement route.

**Architecture:** Preserve the current locked D435 chassis handoff and `auto_insert_160` task. Add a pure D405 horizontal estimator and atomic overlay writer, extend taught-route schema and pre-observation validation, then call the optional D405 correction loop between the fixed waypoint and insertion. Existing IK/path, motion token, timeline, release and route reversal remain authoritative.

**Tech Stack:** Python 3, NumPy, OpenCV, SciPy, existing ARX web controller/trajectory stack, browser JavaScript and Node test runner.

**Spec:** `docs/superpowers/specs/2026-11-19-d405-horizontal-preobserve-design.md`

## Global Constraints

- D405 correction axis is exactly `base_link +Y`; it must not change TCP X, TCP Z, tool rotation, lift, left arm or gripper target.
- Pre-observation insertion axis is `base_link +X`; full insertion is fixed-Z and 160 mm.
- Selected IK branch keeps J6 at `-90 deg` within `3 deg`; J5 software-limit margin is at least `5 deg`.
- D405 convergence is `<0.75 mm`, per-step command is at most `3 mm`, cumulative correction at most `15 mm`, and at most four observations.
- Lateral zero is the existing left-side target: opening center plus opening-left axis times effective width divided by four; nominal demo offset is `42.4 mm`.
- Every post-motion correction requires a newer D405 frame with matching camera/calibration identity and paired arm/body timestamps.
- Hidden rim or battery edges are never synthesized; overlay contains observed evidence only.
- D405 failure is bounded and recorded, then the task intentionally continues as fixed replay without claiming a fresh aperture proof.
- Deployment performs no robot motion, no Git reset and no push.

## Review Focus

- Camera orientation maps base Y to image rows instead of columns: Task 1 tests invariant base-Y sign/magnitude from transformed depth points.
- A stale or calibration-mismatched frame arrives after a correction: Task 1 and Task 3 reject it as motion authorization.
- IK reaches the requested Y but drifts in X/Z/rotation/J6: Task 2 and Task 3 reject the endpoint.
- The demo opening is wider than the battery and center alignment would select the wrong slot: Task 1 tests the literal `42.4 mm` left-side target.
- D405 is absent after navigation: Task 3 proves bounded degradation and continued fixed replay without an unbounded wait.

---

### Task 1: D405 base-Y estimator and evidence overlay

**Files:**
- Modify: `agent/d405_aperture_alignment.py`
- Create: `agent/tests/test_d405_aperture_alignment.py`
- Create: `web/src/d405_alignment_overlay.py`
- Create: `web/tests/test_d405_alignment_overlay.py`

**Interfaces:**
- Produces: `estimate_horizontal_alignment(frame: dict, *, target_offset_mm: float, correction_axis_base=(0.,1.,0.), max_step_mm=3., expected_calibration_sha256: str, after_image_time: float | None=None) -> dict`
- Produces: `write_horizontal_overlay(frame: dict, measurement: dict, output_dir: Path) -> dict`
- Preserves: `estimate_vertical_alignment(frame, *, max_step_mm=3.) -> dict`

- [ ] **Step 1: Write failing estimator tests**

Create synthetic RGB-D fixtures with hand-derived base-frame points and tests named:

- `test_horizontal_alignment_returns_base_y_residual_to_left_quarter_target`
- `test_horizontal_alignment_is_invariant_when_base_y_maps_to_image_rows`
- `test_horizontal_alignment_clamps_step_to_three_mm`
- `test_horizontal_alignment_rejects_stale_frame`
- `test_horizontal_alignment_rejects_wrong_camera_or_calibration`
- `test_horizontal_alignment_rejects_clipped_rim_missing_battery_edge_and_invalid_depth`
- `test_existing_vertical_alignment_contract_remains_available`

The first test must assert literal values for observed rim center, observed battery center, desired `42.4 mm` offset, raw residual and signed step.

- [ ] **Step 2: Run estimator tests and verify RED**

Run: `.venv/bin/python -m unittest agent.tests.test_d405_aperture_alignment -v`

Expected: FAIL because `estimate_horizontal_alignment` does not exist.

- [ ] **Step 3: Implement the horizontal estimator**

Refactor shared visible-edge extraction without changing the old vertical public function. Transform depth-supported rim and battery-edge samples by `base_from_color`, project onto the normalized correction axis, apply the supplied target offset, validate identity/freshness, and return observed pixel/base evidence plus `raw_error_mm` and bounded `step_mm`.

- [ ] **Step 4: Run estimator tests and verify GREEN**

Run: `.venv/bin/python -m unittest agent.tests.test_d405_aperture_alignment -v`

Expected: all estimator tests PASS.

- [ ] **Step 5: Write failing overlay tests**

Test atomic JSON/JPEG output, exact frame identity, red observed rim edges, magenta observed battery edge, yellow correction arrow, unavailable status with no invented edges, and replacement of a prior generation.

- [ ] **Step 6: Run overlay tests and verify RED**

Run: `.venv/bin/python -m unittest web.tests.test_d405_alignment_overlay -v`

Expected: FAIL because `d405_alignment_overlay` does not exist.

- [ ] **Step 7: Implement `write_horizontal_overlay`**

Write `d405-horizontal.jpg` and `d405-horizontal.json` atomically under the supplied directory. JSON exposes frame/calibration identity, observed edges, target offset, raw error, step, cumulative value supplied by the caller, and reason/status.

- [ ] **Step 8: Run Task 1 tests**

Run: `.venv/bin/python -m unittest agent.tests.test_d405_aperture_alignment web.tests.test_d405_alignment_overlay -v`

Expected: all Task 1 tests PASS.

- [ ] **Step 9: Commit Task 1**

```bash
git add agent/d405_aperture_alignment.py agent/tests/test_d405_aperture_alignment.py web/src/d405_alignment_overlay.py web/tests/test_d405_alignment_overlay.py
git commit -m "feat: estimate D405 horizontal cabinet alignment"
```

### Task 2: Taught-route v2 and constrained pre-observation contract

**Files:**
- Modify: `agent/taught_placement.py`
- Modify: `config/taught-placement.json`
- Modify: `web/tests/test_auto_insert_160.py`
- Create: `agent/tests/test_taught_preobservation.py`

**Interfaces:**
- Consumes: current controller model FK and `T_wrist_from_tool`.
- Produces: `load_preobservation_waypoint(path: Path, current_lift: float) -> dict`
- Produces: `validate_preobservation_pose(tool: ndarray, joints: ndarray, joint_limits_deg: ndarray, route: TaughtRoute) -> dict`
- Produces: route fields for waypoint path, target offset, D405 limits, pose tolerances and capture timeout.

- [ ] **Step 1: Write failing route/waypoint tests**

Cover schema version 2, dedicated waypoint path, nominal `42.4 mm` offset, `0.75/3/15/4` D405 limits, `8 s` capture timeout, valid waypoint structure, explicit right arm and closed-gripper target, lift mismatch, insertion-axis error, Z/rotation tolerance, J6 outside `-90±3 deg`, J5 margin below `5 deg`, and insufficient elbow margin.

- [ ] **Step 2: Run Task 2 tests and verify RED**

Run: `.venv/bin/python -m unittest agent.tests.test_taught_preobservation web.tests.test_auto_insert_160 -v`

Expected: FAIL because route v2 fields and pre-observation functions do not exist.

- [ ] **Step 3: Implement route v2 and validators**

Keep `fixed_entry_waypoint` for rollback, add `d405_preobservation_waypoint`, and make validation derive constraints from FK/tool data rather than waypoint metadata. The initial deployed waypoint path is `reports/manual-waypoints/right-d405-preobserve-current.json`.

- [ ] **Step 4: Run Task 2 tests and verify GREEN**

Run: `.venv/bin/python -m unittest agent.tests.test_taught_preobservation web.tests.test_auto_insert_160 -v`

Expected: all Task 2 tests PASS.

- [ ] **Step 5: Commit Task 2**

```bash
git add agent/taught_placement.py config/taught-placement.json agent/tests/test_taught_preobservation.py web/tests/test_auto_insert_160.py
git commit -m "feat: define constrained D405 pre-observation pose"
```

### Task 3: Integrate optional D405 correction into `auto_insert_160`

**Files:**
- Modify: `web/src/fast_visual_level.py`
- Modify: `web/tests/test_visual_chassis_integration.py`
- Create: `web/tests/test_d405_horizontal_flow.py`

**Interfaces:**
- Consumes: Task 1 estimator/overlay and Task 2 route/waypoint validators.
- Produces: `d405_horizontal_correction_target(tool: ndarray, measurement: dict, *, limit_mm: float) -> ndarray`
- Produces: `validate_lateral_endpoint(start_tool: ndarray, end_tool: ndarray, end_joints: ndarray, route: TaughtRoute) -> dict`
- Produces: `FastVisualLevel.auto_d405_horizontal_trim(model, tcp, preobservation) -> dict`
- Preserves: `required_cameras('auto_insert_160') == ()` and current locked handoff.

- [ ] **Step 1: Write failing pure correction tests**

Assert Y-only target translation, rejection of X/Z/rotation drift, explicit `+X` tool axis, fixed-Z insertion request, J6 branch enforcement and cumulative-limit enforcement.

- [ ] **Step 2: Write failing flow tests**

Use complete fakes at controller/capture boundaries to prove the order:

`load_locked_handoff -> preobservation move -> pose validation -> fresh D405 capture -> Y-only correction/reobserve -> fixed-Z +X insertion -> release/reverse`.

Also prove:

- every correction consumes a strictly newer `image_time`;
- D405 unavailable or invalid degrades once within the configured timeout and advances to fixed replay;
- the task snapshot records attempt, raw error, step, cumulative correction, convergence and degradation;
- no D405 path changes lift, left arm, gripper, X, Z or rotation.

- [ ] **Step 3: Run Task 3 tests and verify RED**

Run: `.venv/bin/python -m unittest web.tests.test_d405_horizontal_flow web.tests.test_visual_chassis_integration -v`

Expected: FAIL because the current route moves directly from fixed entry to measured-axis insertion.

- [ ] **Step 4: Implement the correction loop and route ordering**

Parameterize `capture_pair(timeout_s)` for the optional phase, validate calibration hash and timestamps before calling Task 1, write overlay evidence for every attempt, run existing `continuous_correction`/`cabinet_path`, validate the measured endpoint after each move, and insert on literal `[1,0,0]` with `fixed_z=True`.

- [ ] **Step 5: Run Task 3 tests and verify GREEN**

Run: `.venv/bin/python -m unittest web.tests.test_d405_horizontal_flow web.tests.test_visual_chassis_integration web.tests.test_auto_insert_160 -v`

Expected: all Task 3 tests PASS.

- [ ] **Step 6: Commit Task 3**

```bash
git add web/src/fast_visual_level.py web/tests/test_d405_horizontal_flow.py web/tests/test_visual_chassis_integration.py web/tests/test_auto_insert_160.py
git commit -m "feat: correct D405 lateral alignment before insertion"
```

### Task 4: Restore D405 evidence and metrics on the visual-docking page

**Files:**
- Modify: `web/public/visual-docking.html`
- Modify: `web/public/visual-docking.js`
- Modify: `web/public/visual-docking.css`
- Create: `web/tests/visual-docking.test.mjs`

**Interfaces:**
- Consumes: `/camera-overlay/d405-horizontal.json`, `/camera-overlay/d405-horizontal.jpg`, and D405 fields from `/api/fast-level`.
- Produces: visible D405 overlay/status plus Y error, command step, cumulative correction, attempt, convergence/degradation and phase timing.

- [ ] **Step 1: Write failing browser behavior tests**

Import the real module with a controlled DOM/fetch clock and assert:

- D435 and D405 images render independently;
- D405 sequence changes refresh the image URL;
- correction metrics render signed millimetres and degraded reason;
- `d405_horizontal_trim` appears in the timing table;
- the start button remains navigation-owned and disabled;
- no Astra request is sent.

- [ ] **Step 2: Run browser test and verify RED**

Run: `node --test web/tests/visual-docking.test.mjs`

Expected: FAIL because the page has no D405 horizontal overlay elements.

- [ ] **Step 3: Implement the page changes**

Keep the current D435 overlay, reference comparison and timing table. Add one D405 evidence card and correction metrics; do not add a second task-start control.

- [ ] **Step 4: Run Task 4 tests and verify GREEN**

Run: `node --test web/tests/visual-docking.test.mjs web/tests/placement.test.mjs web/tests/cameras.test.mjs`

Expected: all selected browser tests PASS.

- [ ] **Step 5: Commit Task 4**

```bash
git add web/public/visual-docking.html web/public/visual-docking.js web/public/visual-docking.css web/tests/visual-docking.test.mjs
git commit -m "feat: show D405 lateral correction evidence"
```

### Task 5: Full verification, waypoint capture and no-motion deployment

**Files:**
- Create on ARX at deployment: `reports/manual-waypoints/right-d405-preobserve-current.json`
- Deploy only the reviewed files from Tasks 1-4 plus the waypoint/config.
- Do not modify: `/home/arx/LIFT`

**Interfaces:**
- Consumes: fresh right-arm/body feedback only for waypoint recording; no command endpoint.
- Produces: backed-up deployment, restarted web service, current API/camera evidence and a physical-test checklist.

- [ ] **Step 1: Run focused Python and browser suites**

Run:

```bash
.venv/bin/python -m unittest agent.tests.test_d405_aperture_alignment agent.tests.test_taught_preobservation web.tests.test_d405_alignment_overlay web.tests.test_d405_horizontal_flow web.tests.test_auto_insert_160 web.tests.test_visual_chassis_correction web.tests.test_visual_chassis_integration web.tests.test_placement_reference_controller web.tests.test_placement_reference_store -v
node --test web/tests/visual-docking.test.mjs web/tests/placement.test.mjs web/tests/cameras.test.mjs
```

Expected: all focused tests PASS.

- [ ] **Step 2: Run the complete repository test suites**

Run the repository's complete Python unittest discovery and Node test commands, record all pass/fail counts, and separate pre-existing failures from regressions.

- [ ] **Step 3: Obtain a whole-branch review and address Critical/Important findings with RED-GREEN tests**

Expected: no unresolved Critical or Important finding.

- [ ] **Step 4: Record the fixed pre-observation waypoint without motion**

Read fresh right-arm/body feedback twice, require stationary agreement, save joints, wrist pose, lift, gripper target, timestamps and source hashes. Do not send any command.

- [ ] **Step 5: Back up and deploy**

Create a timestamped ARX backup of every destination file, copy only reviewed files, restart only the web service, and retain the rollback directory. Do not push Git.

- [ ] **Step 6: Verify deployment without motion**

Verify process ownership, `/api/state`, `/api/fast-level`, static page, D435 readiness and D405 offline/online reporting. Confirm no motion reservation, chassis command or arm command was issued.

- [ ] **Step 7: Commit deployment metadata locally**

Commit the final source/test state and record remote hashes plus verification output in the execution ledger; do not commit live camera images or runtime receipts.
