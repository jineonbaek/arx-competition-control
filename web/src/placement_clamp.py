"""Establish and verify the right gripper's latched close command before placement."""
import math
import time


def require_closed_target(driver):
    if not driver or driver.get('gripper_target_raw') != 0:
        raise ValueError('右夹爪闭合目标 0 未保持')


def close_for_placement(ctrl, token, driver_instance, guard, *, require_load=False):
    if ctrl.config['arms']['right']['gripper_closed_raw'] != 0:
        raise ValueError('右夹爪闭合配置与放置目标 0 不一致')

    def stationary_driver(state, now):
        if guard(state, now) is not True:
            raise ValueError('当前状态不允许建立夹持')
        driver = ctrl.adapter.trajectory_status('right') or {}
        if (not driver_instance or driver.get('driver_instance') != driver_instance
                or not driver.get('ready') or not driver.get('enabled')
                or driver.get('manual_joint_active') or driver.get('status') not in ('idle','reference_complete','failed')):
            raise ValueError('右臂驱动未就绪，无法建立夹持')
        return driver

    driver = stationary_driver(ctrl.snapshot(), None)
    sent = driver.get('gripper_target_raw') != 0
    if sent:
        def preflight(state, now):
            stationary_driver(state, now)
            return True
        reply = ctrl.submit(dict(type='gripper', arm='right', position=0), token, preflight=preflight)
        if not reply.get('ok'):
            raise ValueError('右夹爪闭合指令未被接受')

    # The object can stop the fingers before the motor reaches zero. Confirm the
    # command separately from a stable measured aperture across fresh samples.
    deadline = time.monotonic() + 3.
    first_stamp = None
    while time.monotonic() < deadline:
        state = ctrl.snapshot()
        driver = stationary_driver(state, None)
        arm = state['arms']['right']
        value = arm['gripper']
        stamp = arm['received']
        in_range = -1.5 < value < -.25 if require_load else -1.5 < value <= .25
        if driver.get('gripper_target_raw') != 0 or not arm['fresh'] or not in_range or not math.isfinite(stamp):
            first_stamp = None
        elif first_stamp is None or abs(value - first_value) > .03 or stamp < first_stamp:
            first_stamp, first_value = stamp, value
        elif stamp - first_stamp >= .35:
            return dict(target_raw=0, feedback_raw=value, close_command_sent=sent, driver_instance=driver_instance)
        time.sleep(.03)
    raise ValueError('右夹爪目标 0 或夹持反馈未确认，机械臂尚未开始本阶段运动')
