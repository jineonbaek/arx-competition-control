import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch, call, Mock
import threading
import tempfile
import struct

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("console_safety_test", ROOT / "scripts/start-console.py")
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


def snapshot():
    return {
        "ros": {"connected": True, "domain_id": "191", "feedback_sources": {
            key: {"status": "fresh", "publishers": 1, "other_publishers": 0}
            for key in ("left", "right", "body")}},
        "capabilities": {key: "" for _, keys in launcher.CHECKS.values() for key in keys},
        "body": {"fresh": True, "height": 0.45},
        "chassis_active": False,
    }


class ConsoleSafetyTests(unittest.TestCase):
    def test_disconnected_can_overrides_fresh_cached_ros_for_every_component(self):
        faults = {"can1": "can1 absent", "can3": "can3 absent", "can5": "can5 absent"}
        with patch.object(launcher, "can_health", return_value=(faults, None), create=True), \
             patch.object(launcher, "camera_fault", return_value=""), \
             patch.object(launcher, "whole_battery_fault", return_value=""):
            status = launcher.component_status(snapshot())
        self.assertIn("can1", status["机械臂"])
        self.assertIn("can3", status["机械臂"])
        self.assertIn("can5", status["底盘"])
        self.assertIn("can5", status["升降"])

    def test_stale_fd_is_rejected_even_if_alias_now_resolves_to_new_device(self):
        self.assertTrue(launcher.adapter_fault(
            "can1", [{"pid": 123, "args": ["slcand", "/dev/arxcan1", "can1"],
                      "serial": [{"path": "/dev/ttyACM0 (deleted)", "identity": [1, 2, 3]}]}],
            [1, 2, 3]))

    def test_new_alias_does_not_hide_changed_device_inode(self):
        self.assertTrue(launcher.adapter_fault(
            "can1", [{"pid": 123, "args": ["slcand", "/dev/arxcan1", "can1"],
                      "serial": [{"path": "/dev/ttyACM0", "identity": [1, 99, 3]}]}],
            [1, 2, 3]))

    def test_matching_open_device_is_healthy(self):
        self.assertEqual(launcher.adapter_fault(
            "can1", [{"pid": 123, "args": ["slcand", "/dev/arxcan1", "can1"],
                      "serial": [{"path": "/dev/ttyACM0", "identity": [1, 2, 3]}]}],
            [1, 2, 3]), "")

    def test_unknown_high_or_moving_lift_is_not_bottom(self):
        sample = {"fault": "", "position": -0.45, "span": 0, "speed": 0}
        self.assertTrue(launcher.bottom_ok(snapshot(), sample))
        for body in [{"fresh": False, "height": 0}, {"fresh": True, "height": 5},
                     {"fresh": True, "height": float("nan")}]:
            state = snapshot(); state["body"] = body
            self.assertFalse(launcher.bottom_ok(state, sample))
        for change in [{"fault": "overload"}, {"span": .1}, {"speed": .3},
                       {"position": -5}]:
            self.assertFalse(launcher.bottom_ok(snapshot(), dict(sample, **change)))
        self.assertFalse(launcher.bottom_ok(snapshot(), None))

    def test_recovery_cannot_stop_a_service_before_bottom_confirmation(self):
        with patch.object(launcher, "stop_owned") as stop:
            with self.assertRaises(RuntimeError):
                launcher.restart_faults(physical_bottom=False)
            stop.assert_not_called()

    def test_recovery_only_selects_faulty_services_and_shared_body_once(self):
        state = snapshot()
        state["capabilities"]["chassis"] = "not ready"
        state["capabilities"]["lift"] = "not ready"
        with patch.object(launcher, "can_health", return_value=(
                {"can1": "", "can3": "", "can5": ""}, None)), \
             patch.object(launcher, "camera_fault", return_value=""), \
             patch.object(launcher, "whole_battery_fault", return_value=""):
            self.assertEqual(launcher.recovery_plan(state), ["body"])


class RecoveryFlowTests(unittest.TestCase):
    def setUp(self):
        self.buses = {"can1": "", "can3": "", "can5": ""}
        self.sample = {"position": -0.45, "span": 0, "speed": 0, "fault": ""}

    def test_lost_web_feedback_uses_existing_ros_owner(self):
        with patch.object(launcher, "state", return_value=None), \
             patch.object(launcher, "native_lift_state", return_value=snapshot()) as native:
            self.assertEqual(launcher.lift_state()["body"]["height"], .45)
            launcher.lift_command("lower")
            self.assertEqual(native.call_args_list, [call(), call("lower")])

    def test_can5_failure_never_issues_blind_lowering_or_stops(self):
        with patch.object(launcher, "can_health", return_value=(
                dict(self.buses, can5="USB disconnected"), None)), \
             patch.object(launcher, "lift_command") as command, \
             patch.object(launcher, "native_lift_state") as native, \
             patch.object(launcher, "stop_owned") as stop:
            self.assertFalse(launcher.prepare_recovery(lambda _: None))
            command.assert_not_called();native.assert_not_called();stop.assert_not_called()

    def test_cancel_before_lowering_never_sends_a_command(self):
        cancel = threading.Event();cancel.set()
        with patch.object(launcher, "can_health", return_value=(self.buses, self.sample)), \
             patch.object(launcher, "lift_state", return_value=snapshot()), \
             patch.object(launcher, "lift_command") as command:
            with self.assertRaises(RuntimeError):
                launcher.prepare_recovery(lambda _: None, cancel)
            command.assert_not_called()

    def test_healthy_bottom_must_remain_stationary_before_recovery(self):
        clock = [0.0]
        def sleep(_):clock[0] += 1
        with patch.object(launcher, "can_health", return_value=(self.buses, self.sample)), \
             patch.object(launcher, "lift_state", return_value=snapshot()), \
             patch.object(launcher, "lift_command") as command, \
             patch.object(launcher.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(launcher.time, "sleep", side_effect=sleep):
            self.assertTrue(launcher.prepare_recovery(lambda _: None))
            self.assertGreaterEqual(clock[0], 1.5)
            command.assert_called_once_with("lower")

    def test_stalled_descent_holds_lift_and_never_restarts(self):
        clock = [0.0]
        high = snapshot();high["body"]["height"] = 5.0
        def sleep(_):clock[0] += 1
        with patch.object(launcher, "can_health", return_value=(self.buses, self.sample)), \
             patch.object(launcher, "lift_state", return_value=high), \
             patch.object(launcher, "lift_command") as command, \
             patch.object(launcher, "stop_owned") as stop, \
             patch.object(launcher.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(launcher.time, "sleep", side_effect=sleep):
            with self.assertRaisesRegex(RuntimeError, "可能受阻"):
                launcher.prepare_recovery(lambda _: None)
            self.assertEqual(command.call_args_list, [call("lower"), call("stop")])
            stop.assert_not_called()

    def test_feedback_loss_during_descent_attempts_hold_not_restart(self):
        with patch.object(launcher, "can_health", side_effect=[
                (self.buses, self.sample), (dict(self.buses, can5="lost"), None)]), \
             patch.object(launcher, "lift_state", return_value=snapshot()), \
             patch.object(launcher, "lift_command") as command, \
             patch.object(launcher, "stop_owned") as stop:
            with self.assertRaisesRegex(RuntimeError, "反馈失效"):
                launcher.prepare_recovery(lambda _: None)
            self.assertEqual(command.call_args_list, [call("lower"), call("stop")])
            stop.assert_not_called()

    def test_bottom_changes_after_confirmation_block_every_stop(self):
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(launcher, "RUNTIME", Path(tmp)), \
             patch.object(launcher, "state", return_value=snapshot()), \
             patch.object(launcher, "lift_state", return_value=snapshot()), \
             patch.object(launcher, "can_health", return_value=(
                 self.buses, dict(self.sample, speed=.5))), \
             patch.object(launcher, "stop_owned") as stop:
            with self.assertRaises(RuntimeError):
                launcher.restart_faults(physical_bottom=True)
            stop.assert_not_called()

    def test_restart_only_body_does_not_touch_healthy_arm_or_can_adapters(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp)/"body";binary.touch()
            with patch.object(launcher, "RUNTIME", Path(tmp)), \
                 patch.object(launcher, "BODY", binary), \
                 patch.object(launcher, "state", return_value=snapshot()), \
                 patch.object(launcher, "lift_state", return_value=snapshot()), \
                 patch.object(launcher, "can_health", return_value=(self.buses, self.sample)), \
                 patch.object(launcher, "owned", return_value=[{"pid": 1}]), \
                 patch.object(launcher, "adapter_records", return_value=[]), \
                 patch.object(launcher, "recovery_plan", side_effect=[["body"], []]), \
                 patch.object(launcher, "stop_owned") as stop, \
                 patch.object(launcher, "ensure_can") as ensure_can, \
                 patch.object(launcher, "start", return_value=Mock(pid=123,poll=lambda:None)), \
                 patch.object(launcher.Path, "is_char_device", return_value=True), \
                 patch.object(launcher, "camera_fault", return_value=""), \
                 patch.object(launcher, "whole_battery_fault", return_value=""):
                launcher.restart_faults(True, selected=["body"])
                stop.assert_called_once_with("body")
                self.assertEqual(ensure_can.call_args.args[0], "can5")
                self.assertEqual((Path(tmp)/"body.pid").read_text(), "123\n")

    def test_wrong_ros_domain_restarts_web_not_healthy_drivers(self):
        state = snapshot();state["ros"]["domain_id"] = "42"
        with patch.object(launcher,"camera_fault",return_value=""), \
             patch.object(launcher,"whole_battery_fault",return_value=""):
            self.assertEqual(launcher.recovery_plan(state), ["web"])

    def test_launch_check_never_starts_or_restarts_hardware(self):
        with patch.object(launcher, "ensure_cameras"), \
             patch.object(launcher, "processes", return_value=[["python", str(launcher.SERVER)]]), \
             patch.object(launcher.Path, "is_file", return_value=True), \
             patch.object(launcher.subprocess, "check_output", return_value=""), \
             patch.object(launcher, "state", return_value=snapshot()), \
             patch.object(launcher, "component_status", return_value={"test": ""}), \
             patch.object(launcher, "ensure_can") as ensure, \
             patch.object(launcher, "start") as start:
            launcher.launch()
            ensure.assert_not_called();start.assert_not_called()


class PassiveCanTests(unittest.TestCase):
    def feedback(self, payload, flags=0):
        clock = [0.]
        class FakeSocket:
            def __init__(self,*_):self.frames=[]
            def setblocking(self,_):pass
            def bind(self,address):
                if address[0]=="can5":
                    self.frames=[(struct.pack("=IB3x8s",0,8,payload), [],flags,None)]
            def recvmsg(self,_):return self.frames.pop(0)
            def close(self):pass
        def select(sockets,*_):
            ready=[s for s in sockets if s.frames]
            if not ready:clock[0]=1.
            return ready,[],[]
        with patch.object(launcher.socket,"socket",side_effect=FakeSocket), \
             patch.object(launcher.select,"select",side_effect=select), \
             patch.object(launcher.time,"monotonic",side_effect=lambda:clock[0]):
            return launcher.motor_feedback()

    def test_own_tx_echo_is_never_accepted_as_motor_feedback(self):
        errors,sample=self.feedback(bytes([0x11,0x7f,0xff,0x7f,0xf0,0,37,38]),
                                    launcher.socket.MSG_DONTROUTE)
        self.assertIn("can5",errors);self.assertIsNone(sample)

    def test_real_motor_overload_is_displayed_and_invalidates_bottom(self):
        errors,sample=self.feedback(bytes([0xe1,0x7f,0xff,0x7f,0xf0,0,37,38]))
        self.assertIn("过载",errors["can5"]);self.assertTrue(sample["fault"])
        self.assertFalse(launcher.bottom_ok(snapshot(),sample))

if __name__ == "__main__":
    unittest.main()
