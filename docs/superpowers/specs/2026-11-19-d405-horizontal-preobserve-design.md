# D405 Horizontal Pre-observation Correction Design

## Intent

Restore D405 as the final close-range visual correction source without undoing the current radar navigation, D435 chassis correction, locked chassis handoff, fixed arm entry, 160 mm insertion, release, reverse withdrawal, timing, or reference-image workflow.

After the chassis is locked, the right arm first moves to one recorded pre-observation waypoint. That waypoint fixes the insertion geometry. D405 then compares the visible red cabinet rim with the held battery's cabinet-facing front edge and may correct only the `base_link` Y coordinate. Height and rotation are never derived from D405.

## Goals

- Preserve the current navigation and D435 chassis-correction handoff.
- Add a fixed right-arm pre-observation waypoint immediately after chassis lock.
- Validate the waypoint against a constrained tool pose:
  - insertion axis is `base_link +X`;
  - TCP Z stays at the waypoint's recorded height;
  - tool rotation stays at the waypoint's recorded rotation;
  - the selected IK branch keeps J6 at `-90 deg` within a configured tolerance;
  - the elbow remains away from a straight-arm singularity and J5 retains software-limit margin.
- Use D405 aligned RGB-D plus its hand-eye transform to compare the red rim's visible inner edges with the battery front edge.
- Convert the comparison to a signed `base_link` Y error, never a raw pixel direction.
- Apply bounded Y-only corrections and reobserve after every correction.
- Draw the red rim edges, battery front edge, correction arrow, raw Y error, commanded step and cumulative correction on the placement page.
- Continue recording planned, actual and cumulative phase durations.
- Keep the existing D435-only/fixed route available for rollback.

## Non-goals

- D405 does not move the chassis.
- D405 does not change TCP X, TCP Z, tool rotation, J6 target, lift, left arm or gripper target.
- D405 does not perform generic battery segmentation or require a complete four-edge battery contour.
- The software does not synthesize hidden rim or battery edges from CAD.
- This change does not run a physical insertion during software deployment.
- This change does not push or reset the dirty ARX repository.

## Selected Architecture

The automatic route becomes:

1. Radar navigation reaches the fixed area.
2. D435 corrects the chassis pose.
3. The chassis is stopped and the locked visual handoff is admitted.
4. The right arm moves to the recorded D405 pre-observation waypoint.
5. The waypoint's full tool pose and joint-margin constraints are validated.
6. D405 captures a fresh paired RGB-D frame.
7. The estimator fits the visible red rim inner edges and the held battery front edge, transforms their depth-supported centers into `base_link`, and returns signed Y error relative to the existing taught left-side target: the opening center plus its left axis times one quarter of the effective opening width. For the nominal 169.6 mm demo opening this offset is 42.4 mm; it is not center-to-center alignment.
8. If the error magnitude is below `0.75 mm`, correction is complete.
9. Otherwise a constrained correction changes only TCP Y, with a maximum `3 mm` command per observation, maximum `15 mm` cumulative correction and maximum four observations.
10. Every motion is followed by a new D405 exposure; pre-motion frames are never reused.
11. The corrected pose is used as the start of a fixed-Z insertion along explicit `base_link +X`, not the measured tool axis.
12. The existing release and exact reached-route reversal return the arm.

The existing `estimate_vertical_alignment()` remains available for old tools. A new horizontal estimator owns the new behavior; callers must choose explicitly, preventing a silent axis change in other flows.

## Coordinate Contract

- World frame: `base_link`.
- Insertion axis: `[1, 0, 0]` within the configured angular tolerance.
- D405 correction axis: `[0, 1, 0]`.
- Locked dimensions during D405 correction:
  - X: unchanged;
  - Z: unchanged;
  - rotation matrix: unchanged;
  - J6 branch: `-90 deg` within tolerance.
- The estimator may rotate/transcode image coordinates internally, but its public result is always a signed base-frame Y error in millimetres.

## Visual Evidence

The executable estimator uses only visible, depth-supported evidence:

- red rim inner-edge samples;
- battery cabinet-facing front-edge samples;
- aligned D405 depth;
- D405 intrinsics supplied with the frame;
- `base_from_color` from the installed hand-eye calibration.

The overlay displays observed edges only. Missing, clipped or depth-invalid evidence is reported as unavailable; hidden edges are not filled from the cabinet or battery model.

## Limits and Failure Behavior

- Converged: `abs(lateral_error_mm) < 0.75`.
- Maximum command per observation: `3 mm`.
- Maximum cumulative correction: `15 mm`.
- Maximum observations: `4`.
- A frame captured before the preceding correction cannot authorize another correction.
- Every authorizing frame must match camera `d405`, the configured camera serial/calibration hash and capture session, be fresh, and be timestamp-paired to the arm and body feedback used for its transform.
- A correction plan is rejected if its endpoint changes X or Z beyond `0.5 mm`, changes tool rotation beyond `0.2 deg`, changes the insertion-axis angle beyond `0.2 deg`, or leaves the configured J6 branch tolerance.
- D405 absence, clipping, invalid depth, invalid calibration identity or exhausted attempts produces a bounded degraded result. It never invents a correction and never waits indefinitely. The route intentionally continues as fixed replay from the pre-observation pose, records the degradation and still runs the existing trajectory/driver checks. This is not a claim of a fresh aperture proof.
- Feedback loss, driver replacement, chassis/lift/left-arm change or trajectory rejection retains the existing stop/hold behavior.

## Interface and Records

- `config/taught-placement.json` references a dedicated pre-observation waypoint and contains the D405 horizontal limits.
- The task snapshot exposes D405 status, observation count, raw Y error, commanded step, cumulative correction, convergence and degradation reason.
- Each run stores D405 measurement JSON and an overlay image tied to the same camera sequence and robot feedback.
- The visual-docking page shows both D435 and D405 evidence while preserving the reference/current D435 comparison and timing table.

## Approaches Considered

### Restore the old vertical-only auto-full route

Smallest code change, but it discards the current navigation and locked handoff and contradicts the new fixed-height requirement.

### Insert D405 Y-only correction into the current locked-handoff route

Selected. It preserves the working current route, assigns one controller to each degree of freedom and makes the new correction independently testable and removable.

### Alternate D435 and D405 full-pose corrections

Rejected because two asynchronous controllers could overwrite each other, oscillate and make the final pose difficult to reproduce.

## Verification

- Unit tests use synthetic RGB-D frames and explicit `base_from_color` transforms to prove the sign and magnitude of base-Y correction, including an image orientation where base Y maps to image rows rather than columns.
- Tests prove clipped rim, missing battery edge, invalid depth and changed camera calibration cannot produce a motor correction.
- Flow tests prove the fixed pre-observation waypoint precedes D405, D405 corrections preserve X/Z/rotation/J6, each correction requires a newer exposure, and all configured limits are enforced.
- Browser tests prove both overlays, correction metrics, phase timing and degraded status render without allowing a second start.
- Deployment verification is limited to source hashes, service/API readiness and non-motion camera state.
- After hardware repair, physical validation proceeds separately: stationary frame, empty-gripper 3 mm correction, held-battery outside correction, shallow insertion/retraction, then full 160 mm placement.
