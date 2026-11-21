# Local Plan F recovery implementation

Spec: ../specs/2026-11-22-grasp-recovery.md

1. Add regression tests for reference-complete settling, diagnostics, legacy
   strict behavior, cancellation, stale feedback and motion before closure.
   Implement opt-in Plan F arrival evidence through planner, executor and mission.
2. Test and implement capture of point 2, measured approach traces, checked return
   and alternate observation postures, durable recovery identity and four attempts.
   Wire production Plan F; reset perception only after confirmed recovery.
3. Verify offline tests, review diff with a fresh reviewer, save local patch,
   changed files and validation record. No deployment or Git push.

Shared interface: monitor -> step -> backend -> mission must distinguish measured
pose_reached from reference_settled; recovery consumes only confirmed approach
traces. Review focus: stale/replayed command identity, late cancellation, unknown
grip, failed stops, recovery reset of evidence, other plan modes, lift/CAD drift.
