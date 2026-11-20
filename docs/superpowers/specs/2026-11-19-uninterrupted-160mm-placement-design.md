# Uninterrupted 160 mm Battery Placement Design

## Intent

Turn the successful manual sequence into one reusable web action: make a small pre-insertion correction, insert the held battery 160 mm without operator pauses, release it, withdraw along the executed route, and return the right arm to its captured start pose.

## Goals

- One button starts the full sequence; no intermediate confirmation or manual continuation.
- Prefer fresh D435 aperture alignment before insertion.
- If D435 is unavailable or stale before any insertion motion, automatically use the captured fixed calibration pose.
- Insert exactly 160 mm along one locked entrance axis.
- Open the right gripper automatically, reverse the actual reached route, and return to the captured start joints.
- Record planned, actual, and cumulative time for every phase.
- Use the repository's existing maximum motion limits: TCP 60 mm/s, TCP acceleration 120 mm/s2, proximal joints 15 deg/s, distal joints up to 50 deg/s, with driver acceleration capped at 18 deg/s2.

## Non-goals

- Do not move the chassis, lift, or left arm.
- Do not use D405.
- Do not continue from an unknown partially inserted state after feedback loss.
- Do not add new low-level motor gain control.

## Selected approach

Extend the existing `FastVisualLevel` `auto_full` route because it already owns the motion token, plans a locked-axis insertion, opens the gripper, reverses the reached joint route, and records a phase timeline.

The start phase first attempts the existing D435 outer alignment. If D435 readiness or localization fails before insertion begins, it loads `reports/manual-waypoints/right-return-20261119-154755.json`, moves to those fixed right-arm joints while retaining the closed gripper, and derives the insertion axis from that calibrated pose. Once engagement starts, no camera fallback or replanning is allowed.

The taught depth becomes 160 mm. The insertion removes the current extra 8 mm/s and 4 mm/s2 cap and uses the already configured repository and driver limits. The return path remains the exact reverse of the reached joint route.

## Alternatives considered

- Pure fixed replay: fastest and independent of cameras, but less reusable when the cabinet shifts.
- D435-only with mandatory fresh vision: best geometric adaptation, but cannot start when D435 is stale.
- Hybrid D435-first with fixed fallback: selected because it keeps visual correction when available and still completes unattended when the camera is unavailable.

## Failure behavior

- Before insertion: stale D435 selects the fixed calibration path automatically.
- After insertion begins: stale robot feedback, driver replacement, chassis/lift/left-arm movement, trajectory rejection, excessive tracking error, or failure to reach the gripper-open target stops the task and records the phase and reason.
- No automatic retry occurs after partial insertion.
- The operator stop button remains available, but the normal sequence contains no manual gates.

## Interface

- The existing visual-docking page button starts the full automatic operation instead of D435-only prepositioning.
- The page continues to show phase, planned time, actual time, cumulative time, D435 overlay, insertion depth, release state, and fallback source.

## Verification

- Unit tests cover fresh-D435 selection, stale-D435 fixed fallback, 160 mm depth, configured maximum speed use, release, route reversal, timing fields, and failure after partial insertion.
- Run focused Python and browser tests, then the full web test suite.
- Deployment verification is limited to process/API readiness unless the user separately requests another physical battery run.
