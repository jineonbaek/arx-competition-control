# Held placement implementation plan
Spec: docs/superpowers/specs/2026-11-16-held-placement.md
- [x] RED/GREEN no-release modes and bounded geometry, default release unchanged.
- [x] Affected fake-transport regressions and final independent review.
- [x] Idle web-only backup/CAS deployment; driver/camera identities unchanged.
- [x] Fresh scene and loaded sweep, align-only attempt if safe; inspect frames before shallow insertion. Record physical result without claiming millimetre accuracy.
Human emergency-stop monitoring and clear motion area confirmed this turn. User requested direct execution without unnecessary questions.
Ruling: explicit no-release overrides original task release. Never race cancellation against release; an error could lose the battery.

Result:76 affected tests passed; independent review depth-cap finding fixed RED→GREEN (whole leading envelope, not center travel). Added nominal loaded sweep with RGB/CAD table inference and between-sample bound. Deployed web-only with CAS backup held-no-release-20261116-022231, drivers/camera unchanged. Actual align_held run moved loaded arm, then first segment joint-arrival timed out. No insertion or release. New stop log preserved gripper target0. User confirmed stopped, grip secure, no contact/noise/shaking. Kept recovery ownership; no automatic retry. Historical replay FK error about6.59mm, not independent physical positioning measurement. Actual placement not completed. Source kept independent and not pushed this turn.
