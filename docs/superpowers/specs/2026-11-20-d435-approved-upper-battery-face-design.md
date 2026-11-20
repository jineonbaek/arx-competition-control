# D435 approved upper battery face design

## Intent

Detect the user-confirmed battery face in the upper-right of the D435 image. The approved real-frame target is the clipped rectangular face near pixels `(480, 0)` to `(639, 165)`, not the lower connection housing previously selected.

## Selected approach

Use a camera-frame depth-plane detector independent of the obsolete battery-pose reference. Search a broad normalized upper-right region, enumerate several depth planes, and select only a large near plane whose normal is nearly camera-facing and whose visible component reaches the top/right image boundary. Join the face's horizontal texture gaps before fitting a clipped rectangle.

The alternative handle detector is not selected for this frame because the handle is occluded and low contrast. Handle localization may be added later for insertion-axis estimation, but it is not required to identify this approved battery face.

## Required behavior

- Match the approved rectangle on the real RGB-D fixture within 12 pixels per visible boundary.
- Reject the lower connection plane, tilted arm planes, missing depth, and upper-right planes that are too small.
- Do not read the old tracked battery pose in the live D435 face-selection path.
- Render the approved measured/fitted face as a magenta rectangle labeled `BATTERY FACE`.
- Do not emit the old `BATTERY FRONT` insertion arrow or claim hidden-space clearance.
- Report depth, plane normal, boundary contacts, confidence, and `insertion_axis_available=false`.
- No robot, gripper, lift, chassis, placement, or depth-enable command is part of this change.

## Verification

The user-approved rectangle is stored as a JSON regression label beside the frozen RGB-D fixture. Synthetic cases cover correct top/right clipping, a larger lower plane, a tilted arm plane, insufficient area, and missing depth. Existing D435/D405/visual-docking focused tests must remain green before deployment.
