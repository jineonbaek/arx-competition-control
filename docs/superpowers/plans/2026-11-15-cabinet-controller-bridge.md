# Cabinet controller bridge implementation plan

> **For agentic workers:** Use superpowers:executing-plans to implement these tasks inline. No robot commands or service changes are part of verification.

**Goal:** Prepare an in-process adapter for the existing right-arm controller, while preserving the physical evidence boundary that currently prevents insertion.

**Architecture:** Keep OfflineCabinetRun and SimulationExecutor unchanged. A separate bridge accepts a trusted in-process physical approval provider and uses the existing controller's reservation and timed-trajectory APIs. No approval provider is installed by default, no HTTP command route is added, and no simulator receipt can authorize this bridge. Missing calibration, loaded stopping coverage or aperture clearance must prevent dispatch.

**Tech Stack:** Python, unittest, existing Controller/FakeAdapter and ROS trajectory contracts.

**Spec:** ../specs/2026-11-15-local-autonomous-cabinet-design.md, sections 2, 4, 5 and G01–G20.

## Global constraints

- Author only in the independent workspace; preserve all existing changes. No commits/pushes or online deployment.
- No camera, CAN or SDK construction in the new module. Tests use fake transports.
- This phase supports right-arm trajectories only: outside alignment, insertion and retreat. It does not control chassis/lift or release the gripper.
- Operator supervision does not create physical calibration or clearance evidence.
- Existing simulator remains simulation-only. Current physical fixed-Z collision remains a dispatch blocker.
- Controller callbacks are an integration seam, not a completed real deployment. Physical approval production and server/state-machine wiring remain unimplemented and must be reported.

## Review focus

- Lost dispatch acknowledgement preserves ownership and an unresolved outcome, even after measured rest.
- Expired/restarted/changed permits stop rather than renew; human cancellation stays latched.
- An accepted stop or reference-complete receipt cannot release ownership before fresh consecutive measured rest.
- Stale feedback cannot suppress an independent right-arm stop request.
- A later plan cannot reuse approvals bound to another trajectory, calibration, grasp or driver.

### Task 1: Approval and right-arm dispatch contract

Files: agent/cabinet_controller_bridge.py; web/tests/test_cabinet_controller_bridge.py; web/src/control.py (optional approved driver binding).

- [x] Write failing tests for absent provider, offline Decision rejection, incomplete gates, unsafe error budget, wrong plan/driver binding and valid fake-controller dispatch.
- [x] Run failing tests, implement immutable approval/policy contracts and existing-controller start/poll calls.

### Task 2: Stop, outcome and measured-rest lifecycle

- [x] Add failing tests for expiry, driver restart, stale feedback stop, late acknowledgement, cancellation, duplicate/gapped feedback, reference completion, timeout and unknown result retention.
- [x] Implement independent stop request and fresh multi-component rest confirmation before token release; no automatic restart or replay.

### Task 3: Verification and deployment boundary

- [x] Run focused tests with both independent and currently installed Controller source, plus existing trajectory/ownership and held/docking regressions. The installed Controller lacks the new atomic driver-binding argument and is intentionally rejected before transport use; this is a checked deployment blocker, not a passing physical integration.
- [x] Synchronize bridge/test/docs and narrow controller change into the author worktree with conflict checks; do not change the active service.
- [x] Record results and remaining physical blockers in docs/cabinet-controller-bridge.md. Agent370 + web431 + browser50 = 851 passing offline tests; no hardware motion or service switch.

## Execution rulings

- Ruling: require an optional expected_driver contract in the prepared Controller and reject the currently installed Controller before bridge use, rather than publishing with a newly fetched epoch. A driver stop increments its epoch; accepting the new epoch would let a stale approval restart motion. Current online source remains unchanged.
- Ruling: serialize handoff and cancellation state commits; keep independent stop dispatch outside that lock. Completion committed before a concurrent cancellation preserves measured-rest proof and reports the cancellation latch for the future parent, which is not wired in this phase.
- Review: five initial reproducible races and two follow-up boundary cases were reproduced with fake transports and fixed with RED→GREEN regressions. Final focused review found no further consequential issue; 32 tests pass locally.
- Current physical collision, missing metric error bounds, unverified loaded stopping and missing production provider remain blocking conditions; this plan does not authorize or claim live placement.
