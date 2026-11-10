import json
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from control import Controller, Rejected

class FakeAdapter:
    def __init__(self): self.calls = []
    def status(self): return {"connected": True, "error": ""}
    def has_subscriber(self, topic, kind): return True
    def lift_ready(self): return True
    def body_calibrated(self): return True
    def arm_ready(self, side): return True
    def arm_stop_ready(self, side): return True
    def arm_stop(self, side): self.calls.append(("arm_stop", (side,)))
    def arm(self, *args, **kwargs): self.calls.append(("arm", args))
    def chassis(self, *args): self.calls.append(("chassis", args))
    def lift(self, *args): self.calls.append(("lift", args))

def configured():
    c = json.loads((Path(__file__).resolve().parents[1] / "config/control.json").read_text())
    c["execution_enabled"] = True
    c["body"].pop("lift_stop_service", None)
    c["body"].pop("calibrated_topic", None)
    for v in c["arms"].values():
        v.pop("command_result_topic", None)
        v.pop("ready_topic", None)
        v.pop("stop_service", None)
        v.update(verified=True, tcp_verified=True, stop_verified=True,
                 joint_limits_deg=[[-90, 90]] * 6, gripper_limits_raw=[-3, 0],
                 max_delta_deg=10, tcp_limits_mm=[[-500, 500]] * 3)
    c["body"].update(lift_verified=True, lift_stop_verified=True,
                     chassis_verified=True, chassis_stop_verified=True,
                     max_linear_m_s=0.1, max_angular_rad_s=0.2)
    return c

class ControlTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.adapter = FakeAdapter()
        self.controller = Controller(configured(), self.adapter, lambda: self.now)
        for side in ("left", "right"):
            self.controller.update_arm(side, [0.] * 6, -1.5, [0.] * 6)
        self.controller.update_body(5., [0.] * 4, 0.2, 0.1, -0.1)

    def test_ros_float64_array_feedback_reaches_vehicle_pose(self):
        import numpy as np
        q = list(np.zeros(6, dtype=np.float64))
        self.controller.update_arm("right", q, -1., q)
        self.controller.update_body(6., list(np.zeros(4, dtype=np.float64)), 0., 0., 0.)
        state = self.controller.snapshot()
        self.assertEqual(state["body"]["height"], 6.)
        self.assertIsInstance(state["arms"]["right"]["joints"][0], np.float64)
        self.assertIsNotNone(state["arms"]["right"]["base_wrist"])

    def test_observation_mode_blocks_every_actuator_without_writes(self):
        self.controller.config["execution_enabled"] = False
        commands = [
            {"type": "arm.joints", "arm": "left", "joints": [0.] * 6},
            {"type": "arm.tcp", "arm": "left", "position_mm": [0.] * 3, "orientation_deg": [0.] * 3},
            {"type": "gripper", "arm": "left", "position": -2.},
            {"type": "arm.stop", "arm": "right"},
            {"type": "chassis", "direction": "forward", "speed": .05, "turn_speed": .1, "sequence": 1},
            {"type": "chassis.stop", "sequence": 2},
            {"type": "lift", "target": 3.},
            {"type": "lift.stop"}]
        for command in commands:
            with self.subTest(command=command), self.assertRaises(Rejected):
                self.controller.submit(command, "A")
        self.assertEqual(self.adapter.calls, [])

    def test_stale_feedback_does_not_become_a_motion_start(self):
        self.now += 2
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "arm.joints", "arm": "left", "joints": [1.] * 6}, "A")
        self.assertEqual(self.adapter.calls, [])

    def test_degrees_map_to_radians_and_preserve_other_arm_and_gripper(self):
        self.controller.submit({"type": "arm.joints", "arm": "right", "joints": [1.] * 6}, "A")
        args = self.adapter.calls[-1][1]
        self.assertEqual(args[0], "right")
        self.assertAlmostEqual(args[1][0], math.pi / 180)
        self.assertEqual(args[2], -1.5)
        self.assertEqual(args[4], 5)
        self.assertEqual(len(self.adapter.calls), 1)

    def test_stop_holds_measured_positions_and_does_not_open_gripper(self):
        self.controller.update_arm("left", [0.1] * 6, -2., [0.] * 6)
        self.controller.submit({"type": "arm.stop", "arm": "left"}, "A")
        self.assertEqual(self.adapter.calls[-1][1][1:3], ([0.1] * 6, -2.))

    def test_non_finite_bool_out_of_range_and_unknown_fields_rejected(self):
        for joints in [[float("nan")] * 6, [True] * 6, [91.] * 6, [0.] * 5]:
            with self.subTest(joints=joints), self.assertRaises(Rejected):
                self.controller.submit({"type": "arm.joints", "arm": "left", "joints": joints}, "A")
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "arm.stop", "arm": "left", "home": True}, "A")
        self.assertEqual(self.adapter.calls, [])

    def test_absolute_joint_targets_inside_limits_are_not_rejected_by_distance(self):
        target = [45., -30., 60., 0., 0., 0.]
        self.controller.submit({"type": "arm.joints", "arm": "left", "joints": target}, "A")
        self.assertEqual(self.adapter.calls[-1][1][1], [math.radians(v) for v in target])

    def test_gripper_zero_is_valid_and_driver_rejection_reaches_http_caller(self):
        self.controller.config["arms"]["left"]["command_result_topic"] = "/web/left/command_result"
        def fail(*args, **kwargs):
            raise Rejected("driver rejected command")
        self.adapter.arm = fail
        with self.assertRaisesRegex(Rejected, "driver rejected command"):
            self.controller.submit({"type": "gripper", "arm": "left", "position": 0}, "A")

    def test_unknown_limits_cannot_enable_real_controls(self):
        self.controller.config["arms"]["left"]["joint_limits_deg"] = None
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "arm.stop", "arm": "left"}, "A")
        self.assertEqual(self.adapter.calls, [])

    def test_tcp_units_and_lift_guard(self):
        self.controller.submit({"type": "arm.tcp", "arm": "left", "position_mm": [100., 0., 0.], "orientation_deg": [90., 0., 0.]}, "A")
        tcp = self.adapter.calls[-1][1][3]
        self.assertAlmostEqual(tcp[0], .1)
        self.assertAlmostEqual(tcp[3], math.pi / 2)
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "lift", "target": 18.0001}, "A")
        self.controller.submit({"type": "lift", "target": 18.0}, "A")
        self.assertEqual(self.adapter.calls[-1], ("lift", (18.0,)))
        self.controller.submit({"type": "lift.stop"}, "A")
        self.assertEqual(self.adapter.calls[-1], ("lift", (5.,)))

    def test_requested_chassis_limits_allow_positive_values_and_reject_overflow(self):
        actual = json.loads((Path(__file__).resolve().parents[1] / "config/control.json").read_text())
        self.assertEqual(actual["body"]["max_linear_m_s"], 2)
        self.assertEqual(actual["body"]["max_angular_rad_s"], 1)
        self.controller.config["body"].update(max_linear_m_s=2, max_angular_rad_s=1)
        for sequence, direction in enumerate(["forward", "left", "yaw-right"], 1):
            self.controller.submit({"type": "chassis", "direction": direction,
                                    "speed": 2, "turn_speed": 1, "sequence": sequence}, "A")
        for sequence, (speed, turn) in enumerate([(0, 1), (-1, 1), (2.0001, 1), (2, 0),
                                                  (2, -1), (2, 1.0001), (float("nan"), 1)], 4):
            calls = len(self.adapter.calls)
            with self.subTest(speed=speed, turn=turn), self.assertRaises(Rejected):
                self.controller.submit({"type": "chassis", "direction": "forward",
                                        "speed": speed, "turn_speed": turn, "sequence": sequence}, "A")
            self.assertEqual(len(self.adapter.calls), calls)
        self.controller.submit({"type": "chassis", "direction": "forward",
                                "speed": .0005, "turn_speed": .0005, "sequence": 20}, "A")

    def test_explicit_brake_still_publishes_when_no_web_motion_is_active(self):
        self.controller.submit({"type": "chassis.stop", "sequence": 1}, "A")
        self.assertEqual(len(self.adapter.calls), 1)
        self.assertEqual(self.adapter.calls[-1][1][:3], (0, 0, 0))
        self.assertEqual(self.adapter.calls[-1][1][3]["height"], 5.)
        self.assertIsNone(self.controller.chassis_owner)
        self.controller.tick()
        self.assertEqual(len(self.adapter.calls), 1)

    def test_chassis_timeout_sends_zero_preserving_body(self):
        self.controller.submit({"type": "chassis", "direction": "left", "speed": .05, "turn_speed": .1, "sequence": 1}, "A")
        self.now += .31
        self.controller.tick()
        args = self.adapter.calls[-1][1]
        self.assertEqual(args[:3], (0, 0, 0))
        self.assertEqual(args[3]["height"], 5.)
        self.assertIsNone(self.controller.chassis_owner)

    def test_stop_wins_over_late_motion_and_other_clients(self):
        self.controller.submit({"type": "chassis.stop", "sequence": 2}, "A")
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "chassis", "direction": "forward", "speed": .05, "turn_speed": .1, "sequence": 1}, "A")
        self.controller.submit({"type": "chassis", "direction": "forward", "speed": .05, "turn_speed": .1, "sequence": 3}, "A")
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "chassis", "direction": "right", "speed": .05, "turn_speed": .1, "sequence": 1}, "B")
        self.controller.disconnected("A")
        self.assertEqual(self.adapter.calls[-1][1][:3], (0, 0, 0))

if __name__ == "__main__": unittest.main()
