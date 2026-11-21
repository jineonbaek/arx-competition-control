"""Check gravity sag at arrival using the arm encoders at a common lift height."""
import time

import numpy as np


J4 = 3
MAX_TRIM_RAD = np.deg2rad(5.0)
MAX_STEP_RAD = np.deg2rad(2.0)


def at_observation(model, tcp, measured, taught, lift, tolerance_mm):
    """Allow existing J4 deflection at the point without retargeting all axes."""
    errors = np.abs(np.asarray(measured) - taught)
    if max(errors) <= np.deg2rad(.1):
        return True
    if np.max(np.delete(errors, J4)) > np.deg2rad(.1) or errors[J4] > MAX_TRIM_RAD:
        return False
    actual = model.wrist_transform(measured, lift) @ tcp
    target = model.wrist_transform(taught, lift) @ tcp
    return (actual[2, 3] - target[2, 3]) * 1000 <= tolerance_mm


def correct_observation_sag(ctrl, tcp, taught, tolerance_mm, guard, move_j4, record):
    model = ctrl.models['right']
    limits = np.deg2rad(ctrl.config['arms']['right']['joint_limits_deg'][J4])
    travel = 0.
    previous = None
    steps = []

    def height(q, lift):
        return float((model.wrist_transform(q, lift) @ tcp)[2, 3] * 1000.)

    while True:
        # Use three distinct stationary feedback messages. Ignore lift differences
        # between teaching and arrival when deciding whether the arm has sagged.
        samples = []
        last_stamp = None
        deadline = time.monotonic() + 1.
        while len(samples) < 3:
            guard()
            state = ctrl.snapshot()
            arm = state['arms']['right']
            if not arm['fresh'] or not state['body']['fresh']:
                raise ValueError('观察点下坠检测缺少实时反馈')
            if last_stamp != arm['received']:
                last_stamp = arm['received']
                q = np.asarray(arm['joints'], float)
                lift = state['body']['height']
                target_z = height(taught, lift)
                samples.append(target_z - height(q, lift))
            if len(samples) < 3:
                if time.monotonic() >= deadline:
                    raise ValueError('观察点关节反馈未更新')
                time.sleep(.06)
        error = float(np.median(samples))
        if not np.isfinite(error):
            raise ValueError('观察点高度反馈无效')
        receipt = dict(axis='J4', target_z_mm=target_z, sag_mm=error,
                       tolerance_mm=tolerance_mm, steps=list(steps))
        record(receipt)
        if error <= tolerance_mm:
            return receipt
        if previous is not None and previous - error < .15:
            raise ValueError('J4 补偿后高度未改善，已保留当前目标')

        driver = ctrl.adapter.trajectory_status('right') or {}
        reference = np.asarray(driver.get('joint_reference_rad'), float)
        if reference.shape != (6,) or not np.isfinite(reference).all() or not driver.get('ready'):
            raise ValueError('J4 补偿需要有效的关节保持目标')
        probe = np.zeros(6)
        probe[J4] = np.deg2rad(.1)
        slope = (height(q + probe, lift) - height(q - probe, lift)) / (2 * probe[J4])
        if not np.isfinite(slope) or abs(slope) < 1.:
            raise ValueError('此姿态下 J4 对高度不敏感')
        direction = np.sign(slope)
        limit_room = min(limits[1] - q[J4], limits[1] - reference[J4]) if direction > 0 else min(q[J4] - limits[0], reference[J4] - limits[0])
        step = min(abs(error / slope), MAX_STEP_RAD, MAX_TRIM_RAD - travel, limit_room)
        if step < np.deg2rad(.02):
            raise ValueError('J4 已到补偿范围，仍有高度偏差')
        delta = direction * step
        goal = q.copy()
        goal[J4] += delta
        if height(goal, lift) <= height(q, lift):
            raise ValueError('J4 修正方向不能抬高末端')
        guard()
        move_j4(q, goal)
        steps.append(float(np.rad2deg(delta)))
        previous = error
        travel += step
