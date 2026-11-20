# Measured 150 mm insertion implementation plan

> Implement inline with superpowers:executing-plans in the existing isolated codex/split-placement-debug worktree. Preserve all unrelated changes. No Git push and no automatic physical trial.

**Goal:** Both independent insertion and competition placement use cabinet depth 150 mm plus the measured signed mouth-to-battery-leading-edge gap, then verify actual battery depth and make bounded supplemental moves.

**Architecture:** Shared callback-based finite insertion controller and RGB-D observation geometry. CurrentAxisPush retains one owner for initial and supplemental moves; FastVisualLevel invokes the same policy with its borrowed competition owner. A completed native travel is never a visual insertion receipt. Existing fixed-travel API remains available explicitly for scripts, while placement buttons use verified insertion.

**Spec:** User requested 150 mm cabinet depth plus gap, post-insertion confirmation, compensation if short, no blocking. Prior requests authorize implementation/deployment, preserve base/lift/left/grip during insertion. Release/return in full placement only after verified entry. User explicitly chose release and original-path exit after attempted insertion even when visual confirmation remains unavailable; retain the unverified result and continue the remaining route with a warning.

## Constraints and review focus

- Initial distance = max(0, 150 - measured leading-edge signed depth). Outside depth is negative; do not add the gap twice or substitute TCP/handle distance.
- Fresh depth-backed battery edges, actual red mouth plane, exact exposure pose/calibration. No disappearance-as-success or FK-only insertion success.
- 2 supplemental moves maximum, <=10 mm each, <=20 mm supplemental total. <=300 mm initial travel. 2.5 s per observation, 8 s total observation waiting; motion remains in background and cancellable.
- Capture timeout cannot cause late worker results to dispatch moves. No compensating when feedback stalls, ownership changes, or observed depth fails to improve.
- After an attempted insertion, release and original-path exit even if visually unverified, per explicit user choice; no release before any push if initial distance is absent. Finite terminal result and visible reason; no infinite visual retry loops.
- Keep legacy operation name auto_insert_160 for compatibility, but depth is measured/configured and successful receipts require verified depth, not literal 160 mm travel.
- Deploy using exact source hashes, backups and web-only restart when idle; never restart arm SDK/drivers or send motion for validation.

## Tasks

- [ ] Tests RED then geometry and bounded policy implementation: signed gap, already partly entered, verified completion, compensation, missing/old/replayed/wrong battery frames, capture timeout, no visual improvement, cancellation.
- [ ] Integrate one-owner CurrentAxisPush automatic mode and independent placement. Update UI to show cabinet depth/gap/planned and actual travel/verified depth/compensation/result. Preserve editable unsaved parameters.
- [ ] Integrate full placement and navigation/debug completion receipts; preserve release and return only after visual verification. Remove fixed 160 mm success comparisons.
- [ ] Focused offline regressions, fresh review, hash-protected deployment and read-only browser/API verification. No physical trial.

## Decisions

- Initial mouth plane may remain frozen when occluded; the post-move battery depth must still be independently observed in a new frame with the same calibration. Missing evidence ends unverified.
- Existing independent settings key insert_distance_mm is migrated to mean cabinet depth, default 150 mm, with schema version 2 so prior 160 mm travel is not silently retained as cabinet depth.
- Stop on a mechanical motion fault; nonblocking means an explicit finite terminal result and responsive UI, not continuing an unknown actuator motion.
