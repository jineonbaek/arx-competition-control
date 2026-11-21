# Insertion contact and seated release

The user approved continuous bounded insertion, contact stopping, and automatic
release only after a near-seat contact and independent D405 seating evidence.
Use the current Python/ROS 2 position trajectory executor. No motor driver
restart, force-mode claim, chassis/platform motion, or blind release is part of
this change.

## Design

- Forward fresh SDK current samples alongside joint feedback. Missing/invalid
  current clears its optional evidence without inventing zero current.
- Monitor actual axial progress, the load-adjusted position reference and
  growth in tracking residual. Learn current baseline/noise only while forward
  travel is observed. Persistent current growth plus position stagnation can
  stop earlier; retain the existing two-second no-progress fallback.
- A typed contact interruption is distinct from cancellation, driver faults,
  stale feedback and geometry errors. Only the typed contact may transition
  from a trajectory to seat observation.
- Stop via the existing measured-position hold, preserve the closed gripper,
  wait for a successful stop response, matching driver/next command epoch and
  fresh stable encoder feedback. Do not reset an interrupted ownership token.
- Preserve only the physically reached prefix for a contact return path.
- A contact within 10 mm of the nominal 150 mm stroke may qualify for release
  only with fresh independent D405 body depth 150 +/- 2 mm and handle-outside
  evidence. Normal no-contact release retains its existing minimum travel.
- Earlier contact enters a held/realignment-required state; no automatic
  lateral motion is commanded while a battery is wedged in the mouth. The
  previously proposed correction branch is represented explicitly, and further
  recovery needs a separately validated retreat path. No contact retries are
  sent into the same blockage.
- Keep the existing maximum travel and speed envelopes. Normal clear travel
  uses the existing continuous plan; vision-authorized compensation remains.
- Record contact evidence, stop confirmation, phase timing and outcome in the
  existing task receipt. Use the same behavior in automated placement and the
  current-axis insertion entry.

## Approach decision

Direct stall-to-release cannot distinguish early wedging from seating. A new
force/impedance controller needs force calibration and SDK validation. The
approved approach extends the existing position executor with contact
monitoring and visual seat confirmation. Current thresholds are conservative
relative-change heuristics, not calibrated force limits or physical proof.

## Verification and deployment

Use synthetic feedback and real state-machine tests without ROS command
transport. Test normal progress, current spikes, static load, missing/stale
feedback, completed references, early/end contacts, failed stop acknowledgments,
manual cancellation, and truncated reverse routes. Deploy only changed files
against matching remote hashes, back up originals, and reload the web process
only when no motion task owns it. No physical insertion test in this change.
