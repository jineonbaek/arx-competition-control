# Execution ledger - plan: docs/superpowers/plans/2026-11-21-insertion-contact.md

Ruling: User's “执行” authorizes the proposed implementation; no repeated design
approval is requested. Work uses an isolated snapshot of current dirty ARX files.

Ruling: Early jam recovery remains an explicit held/realignment-required branch.
Blind lateral recovery inside the cabinet has no certified retreat path; no new
unvalidated recovery motion is invented as part of contact detection.

Pre-flight: telemetry supplies optional current to the pure monitor; the typed
exception is consumed by the motion executor and then the seating state machine.
Stop confirmation does not change Controller's existing manual-stop semantics.

Baseline: current ARX snapshot HEAD 3de748293b0d0b72f3191381b2c6361a0ac187de,
including working changes. Focused baseline tests running in isolated ARX copy.


Implementation and review:
- Current feedback, load-adjusted reference/progress monitor, stop acknowledgement,
  fresh settling, reached-only reverse prefix and D405 seated release implemented.
- Early contact holds with realign_required. No automatic retreat or recovery was
  added; therefore a wedged attempt can still require intervention.
- Reviewer findings fixed: bind contact to issued instance/request/epoch; preserve
  fault reasons; apply both 140 mm and 160 mm seat-zone bounds; retain ownership
  at the shared Controller token so competition/debug wrappers cannot clear it.
- Review after fixes: no remaining findings. No hardware commands were sent.
- Concurrent remote D405 recovery and T2 lifecycle edits were preserved in the
  isolated baseline; merged contact changes do not overwrite them.

Validation:
- RED records: contact-red.log, integration-red.log, review-red.log, owner-red.log.
- Focused final: 154 tests passed, including 21 contact tests and the real current
  insertion state machine with contact, early jam, unknown stop and reverse exit.
- Broad web comparison: 767 candidate / 746 baseline tests; the same 31 failure
  entries in both snapshots. One of those was an incomplete copied T2 page asset;
  refreshing that existing production page made its focused rerun pass.
- Broad agent comparison: 399 tests in each; the same 4 existing failure entries.
- Full suites are not green. No new failure signatures compared with baseline.
- Test fixtures explicitly use the fake in-process chassis rather than live ROS.
- Current-change heuristics are not calibrated force limits. Physical contact,
  holding force and insertion reliability have not been validated on hardware.

Deployment: prepared for hash-checked install and idle web-only reload; see the
separate deployment manifest for the actual outcome. No remote Git push.
