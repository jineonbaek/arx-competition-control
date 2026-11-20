# D435 Partial Battery Depth Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect the partially off-screen black battery in the real D435 RGB-D capture and render a depth-supported contour and axis without confusing the arm or background for the battery.

**Architecture:** Keep kinematic tracking as the spatial prior, replace the color-only face gate with a deterministic depth-plane observation that accepts image-boundary clipping, and preserve the legacy gray-face path as a fallback. Separate measured contour evidence from model-completed edges in metadata and rendering.

**Tech Stack:** Python 3, NumPy, OpenCV, unittest, existing D435 RGB-D capture and ARX kinematic model.

**Spec:** `docs/superpowers/specs/2026-11-20-d435-partial-battery-depth-detection-design.md`

## Global Constraints

- No arm, gripper, lift, chassis, or automatic placement commands.
- Reject observations without paired depth, sufficient planar support, expected-depth agreement, and proximity to the kinematic projection.
- Completed off-screen geometry is visualization only and must not become motion authorization.
- Preserve existing cabinet-rim detection and the complete gray-face fallback.
- Back up the remote untracked source before deployment.

## Review Focus

- A large black arm surface near the predicted ROI must not win over the expected battery-depth component.
- A plane at the wrong depth must return unavailable rather than a plausible-looking cuboid.
- A valid battery touching the top and right image boundaries must remain detectable.
- Missing/zero/NaN depth must return unavailable without exceptions or inferred evidence.
- A complete legacy gray battery face must retain its existing four-edge result.

---

### Task 1: Import the active detector and pin the real failure

**Files:**
- Create: `web/src/d435_live_overlay.py`
- Create: `web/tests/test_d435_live_overlay.py`
- Create: `web/tests/fixtures/d435-partial-black-battery-frame.npz`

**Interfaces:**
- Consumes: the exact active remote detector, unit tests, and cached paired RGB-D frame.
- Produces: a reproducible regression test using the real frame and the current `battery_top_face(...)` API.

- [ ] Copy the exact active remote source and tests into the clean local branch and verify their SHA-256 hashes match the captured originals.
- [ ] Add the captured frame as a test fixture stripped to fields needed for detection (`rgb`, `xyz`, predicted center `(598, 237)`, expected depth `0.653574467`).
- [ ] Add `test_partial_black_battery_at_right_boundary_is_depth_detected`, asserting a non-empty observation whose visible support contains the predicted center neighborhood and touches the right boundary.
- [ ] Run the single test and verify it fails because the existing gray HSV detector returns `None`.

### Task 2: Implement bounded partial-plane observation

**Files:**
- Modify: `web/src/d435_live_overlay.py`
- Modify: `web/tests/test_d435_live_overlay.py`

**Interfaces:**
- Consumes: `rgb`, organized `xyz`, predicted center/depth, and projected cuboid pixels.
- Produces: `battery_depth_face(rgb, xyz, expected_px, expected_depth_m, projected_pixels=None) -> dict | None` with measured contour, measured axis support, visible fraction, boundary contacts, method, and confidence fields.

- [ ] Add synthetic failing tests for right-edge crop, top/right crop, wrong-depth plane, arm-like thin distractor, insufficient area, and missing depth.
- [ ] Run the focused tests and verify each new behavior fails for the intended missing detector behavior.
- [ ] Implement deterministic ROI clipping, expected-depth banding, connected-component selection, plane RANSAC, compactness/extent gates, and boundary-aware visible contour extraction.
- [ ] Keep `battery_top_face(...)` as fallback for complete gray faces.
- [ ] Run the focused tests and real-frame regression; require all pass.

### Task 3: Integrate rendering and metadata

**Files:**
- Modify: `web/src/d435_live_overlay.py`
- Modify: `web/tests/test_d435_live_overlay.py`
- Modify: `web/tests/visual-docking.test.mjs` only if the metadata display contract requires it.

**Interfaces:**
- Consumes: the partial-plane observation and existing tracked cuboid projection.
- Produces: measured contour rendering plus inferred clipped edges, battery axis, and metadata fields `method`, `visible_fraction`, `boundary_contacts`, and `confidence`.

- [ ] Add failing render tests that distinguish measured solid contour from inferred model completion and expose the new metadata without claiming hidden-space clearance.
- [ ] Integrate the depth observation into `LiveOverlay._run()` and `render()`; retain existing unavailable behavior on rejection.
- [ ] Run Python and browser focused suites and verify all pass.

### Task 4: Deploy and inspect without motion

**Files:**
- Deploy: `web/src/d435_live_overlay.py`
- Deploy: `web/tests/test_d435_live_overlay.py`
- Deploy: `web/tests/fixtures/d435-partial-black-battery-frame.npz`

**Interfaces:**
- Consumes: locally verified files.
- Produces: matching remote hashes, passing remote focused tests, restarted web service, and a refreshed D435 overlay generated without robot commands.

- [ ] Back up the three remote targets under a timestamped deployment-backup directory.
- [ ] Upload files and verify local/remote SHA-256 equality.
- [ ] Run remote D435 overlay and related visual-docking tests.
- [ ] Restart only the web service if required; do not start placement or motion.
- [ ] If fresh robot/depth feedback is unavailable, run the detector directly on the cached paired fixture and label the result offline-only.
- [ ] Inspect the generated overlay and metadata; report measured contour, inferred edges, rejection/confidence state, and any remaining lack of live physical proof.
