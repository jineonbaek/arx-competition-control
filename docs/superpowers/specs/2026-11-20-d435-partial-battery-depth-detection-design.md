# D435 partial battery depth detection design

## Intent

Make the D435 grasp/placement overlay detect the held black battery when its upper or right edges are outside the image, as in the captured frame whose model-predicted center is near pixel `(598, 237)`. The result must provide a useful visible contour and orientation without confusing the black robot body or background for the battery.

## Goals

- Use the paired D435 depth image as the primary evidence instead of the existing light-gray HSV mask.
- Search only near the battery location predicted from the recorded grasp pose and current right-arm/lift state.
- Fit the visible battery plane even when it touches an image boundary.
- Use the calibrated battery envelope to complete clipped display geometry and derive the battery axis.
- Preserve the existing complete gray-face detector as a compatible fallback.
- Add the real cached RGB-D frame as a regression fixture and verify that the old implementation fails while the new implementation detects it.

## Non-goals and safety boundary

- This change does not command the arm, gripper, lift, or chassis.
- A completed off-screen outline is visualization/model inference, not proof that hidden space is clear.
- The detector must return unavailable when predicted pose, paired depth, planar support, depth agreement, or visible area is insufficient.
- This change does not alter cabinet-rim detection or motion authorization.

## Approaches considered

1. **Depth-first partial-plane detection (selected).** Segment depth around the kinematically predicted battery projection, fit the dominant local plane, reject candidates inconsistent with expected depth and projected battery size, and clip/reconstruct the calibrated envelope. This handles dark and cropped batteries with bounded false-positive risk.
2. **Widen dark-color thresholds.** Smallest code change, but the black arm, battery, and background overlap strongly in color, so false positives are unacceptable.
3. **Train an object detector.** Potentially useful later, but the current sample set is too small and it would not by itself provide metric pose from paired depth.

## Detection design

1. Project the tracked battery cuboid and its center into the D435 image using the existing arm/lift reference transform.
2. Build a bounded ROI from the projected cuboid plus a small pixel margin, clipped to the image.
3. Keep valid depth pixels within a tight band around the predicted battery depth.
4. Fit a plane with deterministic RANSAC. Require a minimum visible pixel count, inlier ratio, compact connected support, and normal/depth agreement.
5. Extract the visible support contour. Accept contours touching the image boundary; reject thin arm-like components and components whose centroid is too far from the predicted projection.
6. Use the calibrated battery envelope and tracked pose to supply hidden corners. Snap only the visible face translation/plane alignment to measured depth; retain the tracked orientation unless measured visible edges provide a stronger, consistent constraint.
7. Render the measured visible contour distinctly from inferred clipped edges. Metadata records `depth_partial_plane`, visible fraction, boundary contacts, and confidence/rejection reason.

## Verification

- Regression test from the captured `frame.npz`: current detector returns no quad; the new detector returns a depth-supported partial battery observation near the predicted center.
- Synthetic tests for a complete face, right-edge crop, top/right crop, black robot-arm distractor, wrong-depth plane, insufficient area, and missing depth.
- Existing D435 overlay, D405, visual-docking, placement, and camera tests remain green.
- Deploy only after local and remote focused tests pass; refresh the overlay without issuing robot commands and visually inspect the resulting contour.

## Source handling

The active remote `d435_live_overlay.py` and its tests are currently untracked on the ARX checkout and absent from the local branch. Their exact remote versions will first be copied into the local branch as the baseline, then changed through regression tests. The remote originals will be backed up before deployment.
