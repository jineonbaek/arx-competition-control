# Cabinet Offline Chain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Produce an offline-verifiable D435-primary placement preview without trusting unconfigured depth or old robot state.

**Architecture:** RGB-only immutable capture feeds pure CAD PnP, then the existing fixed-world-Z geometry and original TCP chain. Read-only cache validation clears stale coordinates. A replay CLI shares the analyzer and cannot access motor transport.

**Tech Stack:** Existing Python, unittest, NumPy, OpenCV and Node test runner; no new dependencies.

**Spec:** `docs/superpowers/specs/2026-11-15-cabinet-offline-chain-design.md`

## Global Constraints

- No physical test, control command, gripper release, driver restart or push.
- D435 primary, D405 auxiliary; preserve original TCP and nominal D435 status.
- Work in the independent placement source; do not change protected projects or running deployment.
- All output executable=false; point-cloud collision evidence is unavailable.

## Review Focus

- Unsupported or mismatched intrinsics must not create metric coordinates.
- Planar ambiguity and nominal camera extrinsics must remain visible to the consumer.
- Changed joints/lift/gripper, command epoch, source bytes or stale feedback must clear cached coordinates.
- Polling and model switches must not revive previous coordinates or let D405 override D435.
- Offline replay must remain hardware independent and preserve historical timestamps.

### Task 1: RGB capture and metric CAD preview

**Files:** Create `agent/cabinet_rgb_geometry.py`, `web/src/cabinet_capture.py`; modify `web/src/cabinet_preview.py`; add corresponding unittest files.

**Interfaces:** `localize_rgb(frame, profile, now=None)` produces the existing window axes/corners plus camera-relative pose candidates, reprojection and localization source. `CabinetColorCapture.capture()` returns an immutable RGB+feedback frame without depth leases.

- [x] Add tests proving depth-free pose, bad identity/intrinsics rejection and distortion projection.
- [x] Run RED, implement capture and localization, run GREEN.
- [x] Integrate with preview, label nominal calibration and unavailable environment evidence, preserve fixed Z and existing TCP.

### Task 2: Invalidation through server and browser

**Files:** Modify `web/src/cabinet_preview.py`, `web/src/server.py`, `web/public/js/cabinet.js`, `web/public/placement.html`, tests.

**Interfaces:** `CabinetPreview.snapshot(now=None)` returns only current results with valid source/state bindings, otherwise a stale image-only result.

- [x] Add RED tests for stale age, pose/gripper/epoch/source changes and browser disconnect/model response races.
- [x] Implement invalidation and clear visible path/coordinates on failures.
- [x] Run Python and browser suites with no motor calls.

### Task 3: Offline replay and evidence

**Files:** Create `scripts/replay_cabinet.py`, replay tests, and `docs/cabinet-offline-chain.md`.

**Interfaces:** CLI consumes NPZ and `--model demo|competition --output DIR`; reports historical age and unverified execution prerequisites, writes JSON/JPEG, never controls/captures hardware.

- [x] Test CLI on a generated frame before implementation, then implement and run GREEN.
- [x] Replay archived real D435 capture, verify images and fixed-Z result or precise rejection.
- [x] Run full available agent/web/browser suites, review changes, sync source-only to isolated remote worktree with backups if connection permits. Do not deploy or restart services.

Implementation and local checks complete. Full host regression and final evidence ledger are recorded separately; no live deployment. Review also required pinning loaded robot-model byte identity in web/src/handeye_column.py with web/tests/test_cabinet_model_sources.py.
