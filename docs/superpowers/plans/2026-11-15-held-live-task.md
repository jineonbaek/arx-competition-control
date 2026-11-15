# Held cabinet live task implementation plan

Goal: connect the authenticated start/status/cancel route to ControllerBridge and a server-side D435 planner, with5mm outside-only arrival tolerance.
Architecture: CabinetExecutionTask owns one real segment, worker polling and cancellation. Default planner validates the current D435 result before producing a timed right-arm approach/insert segment. It does not convert simulator evidence into physical permission. No physical provider is installed without actual acceptance.
Spec: docs/superpowers/specs/2026-11-15-held-live-task-design.md

- [x] Reproduce RED for real task start→ControllerBridge dispatch using only fake transport; test missing proof, 5mm FK arrival, cancel/restart and unknown outcomes.
- [x] Implement task and protected HTTP routes, status/UI integration, preserve no-target/no-enable payloads and no release/chassis/lift path.
- [x] Verify current D435 localization read-only, full affected suites and newer runtime candidate compatibility; perform one independent code review.
- [x] Deploy only idle web/preview sources with source backup/CAS; preserve camera/driver process identities. Call live start once; if geometry/evidence blocks, report the actual blocker and no physical success claim.

User opted out of further design questions and requested direct execution. No new hardware activation or collision bypass is authorized by the5mm tolerance. Prior push authorization persists, but source publication follows verification only.


Execution record:14 task fake-transport tests +1 HTTP test,9 preview tests,51 browser tests and132 newer-runtime candidate tests passed. Two reviewer findings and three additional lifecycle/context regressions watched RED→GREEN. Live start returned current fixed-Z aperture collision before reservation; epoch0→0, no physical actuation. WebPID11663; driver/camera identities preserved. Production physical provider remains missing; full physical placement remains disabled, not completed.

Ruling:5mm applies to outside TCP arrival only, not negative aperture clearance or a fabricated physical accuracy bound — the79mm slot and75mm battery leave only2mm nominal vertical clearance per side — applying5mm to insertion can cause contact. This restriction was stated to the user before implementation.

Initial deployment rolled back on4s timeout while repeated preview IK was running. Capture-only geometry path and90s deployment wait prevent duplicate IK. Known no-motion interrupted planning receipt archived with unchanged epoch/feedback evidence; no physical unknown cleared.

Full suite uncovered an existing competition fixture race;60ms scheduling delay reproduced it. Only the fixture waits for actual module start now; production competition source unchanged. Final suite result recorded in report and branch history.
