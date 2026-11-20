# D435 Approved Upper Battery Face Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect and render the user-approved upper-right battery face without using the incorrect tracked battery calibration.

**Architecture:** A deterministic multi-plane RGB-D detector scans a normalized upper-right ROI, selects a camera-facing boundary-clipped plane, joins its horizontal texture gaps, and fits a clipped rectangle. Live rendering consumes only this observation and exposes no insertion axis.

**Tech Stack:** Python 3, NumPy, OpenCV, unittest, existing D435 paired RGB-D capture.

**Spec:** `docs/superpowers/specs/2026-11-20-d435-approved-upper-battery-face-design.md`

## Global Constraints

- No motion or depth-enable command.
- No dependency on the old battery reference pose in live face selection.
- No insertion-axis or hidden-space claim.
- User-approved real-frame bounds are `(480, 0)` to `(639, 165)` with 12-pixel boundary tolerance.

## Review Focus

- Larger lower housing plane must not win.
- Camera-facing background or body planes outside the upper-right clipped target must not win.
- Horizontal texture gaps must not split the battery into an incorrect E-shaped contour.
- Missing/NaN depth must return unavailable.
- Live metadata must not expose the old insertion arrow.

---

### Task 1: Pin the approved label and RED cases

**Files:**
- Create: `web/tests/fixtures/d435-approved-upper-battery-face.json`
- Modify: `web/tests/test_d435_live_overlay.py`

- [ ] Add the approved quad and source-image provenance.
- [ ] Add real-frame and synthetic tests for the approved plane and rejection cases.
- [ ] Run the tests and verify failure because `battery_upper_face` does not exist.

### Task 2: Implement upper-face detection and live evidence

**Files:**
- Modify: `web/src/d435_live_overlay.py`
- Modify: `web/tests/test_d435_live_overlay.py`

- [ ] Implement `battery_upper_face(rgb, xyz) -> dict | None` with multi-plane RANSAC and boundary/normal/area gates.
- [ ] Implement `battery_upper_face_evidence(rgb, xyz) -> dict` with no insertion axis.
- [ ] Replace the live battery-pose/reference selection with the new evidence.
- [ ] Render the fitted rectangle and honest metadata.
- [ ] Run the real-frame, synthetic, D435, D405, and browser focused suites.

### Task 3: Deploy without motion

**Files:**
- Deploy: `web/src/d435_live_overlay.py`
- Deploy: `web/tests/test_d435_live_overlay.py`
- Deploy: `web/tests/fixtures/d435-approved-upper-battery-face.json`

- [ ] Back up current remote targets.
- [ ] Upload and verify hashes.
- [ ] Run remote focused tests.
- [ ] Restart only the web service and verify HTTP 200 with no motion reservation or command epoch change.
- [ ] Generate an offline overlay from the frozen fixture and inspect the approved rectangle.
