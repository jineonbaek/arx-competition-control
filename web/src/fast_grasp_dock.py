"""Single T1 forward pulse followed by measured standstill, before wrist capture."""
import time
import uuid

from fast_grasp_start import check_start
from navigation_view import read_snapshot
from navigation.polarbear_adapt.chassis_command import WebChassis
from navigation.polarbear_adapt.mppi_trial.stationary import StopWindow


class FastDock:
    policy = 't1_short_pulse'

    def __call__(self, controller, token, store):
        from agent.chassis_retreat import _PoseReader
        if controller.config.get('chassis_backend') == 'ros':
            controller.chassis.session(token, 'navigation').ensure()
        reader = _PoseReader()
        try:
            # The reader initializes the navigation driver's module path.
            from t1_contact_runtime import DURATION, SLOW_SDK
            output = WebChassis(source_id='navigation-' + uuid.uuid4().hex,
                                submit=lambda cmd: controller.submit(cmd, token))
            output.native_sdk = output.sdk_units = True
            def check_origin():
                check_start(store.state['battery_task_settings']['fast_reference'], read_snapshot())
                robot = controller.snapshot()
                if robot.get('chassis_active') or not robot.get('nav_sdk_ready') or not robot['body']['fresh']:
                    raise ValueError('底盘当前未就绪或正被其他操作使用')
            return approach(reader, output, SLOW_SDK, DURATION,
                            lambda: store.cancelled or not controller.motion_valid(token), check_origin)
        finally:
            reader.close()


def approach(reader, output, effort, duration, cancelled, check_origin, *, clock=time.monotonic, sleep=time.sleep):
    def check():
        if cancelled():
            raise ValueError('贴靠已取消')

    def stationary():
        window = StopWindow(after=reader.now())
        deadline = clock() + 5.
        while clock() < deadline:
            check()
            sample = reader.read()
            if sample is not None and window.add(sample[2], sample[1]):
                return sample
            sleep(.02)
        raise ValueError('雷达未确认底盘停稳')

    sample = stationary()
    check_origin()
    command = (effort, 0., 0.)
    reader.check_motion(command, sample[2])
    check()
    deadline = clock() + duration
    try:
        # A lost acknowledgement may mean accepted motion. Never replay this pulse.
        output.send(command, ttl_s=duration)
        while clock() < deadline:
            check()
            sample = reader.read()
            if sample is None:
                raise ValueError('贴靠时雷达反馈中断')
            reader.check_motion(command, sample[2])
            sleep(min(.02, max(0., deadline - clock())))
    finally:
        output.end_output()
    _, odom, stamp = stationary()
    return dict(status='stopped', stopped_at=time.time(), stop_source_stamp=float(stamp),
                stopped_odom=list(map(float, odom)), source='t1_short_pulse', pulse_s=duration)
