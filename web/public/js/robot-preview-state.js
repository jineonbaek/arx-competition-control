// Read-only URDF display mapping. SDK radians are never clamped to CAD limits.
export const LIFT_RAD_PER_M = 41.54;

export function previewState(snapshot, serverNowS) {
  const fresh = part => snapshot?.ros?.connected === true && part?.fresh === true &&
    Number.isFinite(serverNowS) && Number.isFinite(part?.timestamp) &&
    Number.isFinite(part?.age_s) && part.age_s >= 0 && part.age_s <= 1 &&
    serverNowS - part.timestamp >= -.15 && serverNowS - part.timestamp <= 1.5;
  const arm = side => {
    const part = snapshot?.arms?.[side];
    const valid = fresh(part) && Array.isArray(part.joints) &&
      part.joints.length === 6 && part.joints.every(Number.isFinite);
    const config = snapshot?.limits?.arms?.[side];
    const open = config?.gripper_open_raw, closed = config?.gripper_closed_raw;
    const gripperValid = fresh(part) && Number.isFinite(part.gripper) &&
      Number.isFinite(open) && Number.isFinite(closed) && open !== closed;
    // Map measured opening fraction to the CAD stroke; millimeters need calibration.
    const ratio = gripperValid ? Math.max(0, Math.min(1, (part.gripper - closed) / (open - closed))) : null;
    return {valid, joints: valid ? part.joints.slice() : null,
      gripperValid, fingerM: gripperValid ? ratio * .044 : null};
  };
  const body = snapshot?.body;
  const valid = fresh(body) && Number.isFinite(body.height);
  return {left: arm('left'), right: arm('right'),
    lift: {valid, rad: valid ? body.height : null, positionM: valid ? body.height / LIFT_RAD_PER_M : null}};
}

export function applyPreviewState(robot, value) {
  robot.links.body4.visible = value.lift.valid;
  if (value.lift.valid) robot.joints.joint4.setJointValue(value.lift.positionM);
  for (const [side, first] of [['left', 11], ['right', 21]]) {
    robot.links[side + '_link' + first].visible = value[side].valid;
    if (value[side].valid)
      value[side].joints.forEach((angle, i) => robot.joints[side + '_joint' + (first + i)].setJointValue(angle));
    for (const index of [first + 6, first + 7]) {
      robot.links[side + '_link' + index].visible = value[side].valid && value[side].gripperValid;
      if (value[side].valid && value[side].gripperValid)
        robot.joints[side + '_joint' + index].setJointValue(value[side].fingerM);
    }
  }
}
