import copy
import json
import math
from pathlib import Path
import unittest

from test_control import FakeAdapter
from control import Controller, Rejected

ROOT = Path(__file__).resolve().parents[1]


class ManufacturerLimitTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / "config/control.json").read_text())
        self.sources = json.loads((ROOT / "config/limit-sources.json").read_text())

    def controller(self, max_delta=360):
        config = copy.deepcopy(self.config)
        config["execution_enabled"] = True
        for arm in config["arms"].values():
            arm["verified"] = True
            arm["max_delta_deg"] = max_delta
        adapter = FakeAdapter()
        controller = Controller(config, adapter, lambda: 100.)
        for side, arm in config["arms"].items():
            middle = [math.radians((lo + hi) / 2) for lo, hi in arm["joint_limits_deg"]]
            controller.update_arm(side, middle, -1., [0.] * 6)
        controller.update_body(5., [0.] * 4, 0., 0., 0.)
        return controller, adapter

    def test_deployed_ranges_stay_inside_manufacturer_table_and_tcp_remains_off(self):
        self.assertTrue(all(a["verified"] for a in self.config["arms"].values()))
        self.assertEqual(self.config["body"]["lift_limits_raw"], [0, 18])
        for arm in self.config["arms"].values():
            self.assertTrue(arm["verified"])
            self.assertFalse(arm["tcp_verified"])
            self.assertTrue(arm["stop_verified"])
            for actual, original in zip(arm["joint_limits_deg"], self.sources["joint_software_limits_rad"]):
                expected = [math.ceil(math.degrees(original[0]) * 10) / 10,
                            math.floor(math.degrees(original[1]) * 10) / 10]
                self.assertEqual(actual, expected)
                self.assertGreaterEqual(math.radians(actual[0]), original[0])
                self.assertLessEqual(math.radians(actual[1]), original[1])
            self.assertLessEqual(math.radians(arm["max_delta_deg"]), .1)
            self.assertEqual(arm["gripper_limits_raw"], [-3.14, 0])
            self.assertEqual(arm["gripper_open_raw"], -3.14)
            self.assertEqual(arm["gripper_closed_raw"], 0)

    def test_every_axis_on_both_arms_accepts_bounds_and_rejects_outside_before_publish(self):
        controller, adapter = self.controller()
        for side, arm in self.config["arms"].items():
            middle = [(lo + hi) / 2 for lo, hi in arm["joint_limits_deg"]]
            for i, pair in enumerate(arm["joint_limits_deg"]):
                for edge in pair:
                    target = middle[:]
                    target[i] = edge
                    controller.submit({"type": "arm.joints", "arm": side, "joints": target}, "test")
                    self.assertEqual(adapter.calls[-1][1][0], side)
                for outside in [pair[0] - .01, pair[1] + .01]:
                    target = middle[:]
                    target[i] = outside
                    count = len(adapter.calls)
                    with self.assertRaises(Rejected):
                        controller.submit({"type": "arm.joints", "arm": side, "joints": target}, "test")
                    self.assertEqual(len(adapter.calls), count)

    def test_absolute_joint_goals_and_gripper_intersection_are_enforced(self):
        controller, adapter = self.controller(max_delta=5.7)
        for side in ("left", "right"):
            middle = [(lo + hi) / 2 for lo, hi in self.config["arms"][side]["joint_limits_deg"]]
            target = middle[:]
            target[0] += 5.7
            controller.submit({"type": "arm.joints", "arm": side, "joints": target}, "test")
            target[0] += .01
            controller.submit({"type": "arm.joints", "arm": side, "joints": target}, "test")
            for value in [-3.14, 0]:
                controller.submit({"type": "gripper", "arm": side, "position": value}, "test")
                self.assertEqual(adapter.calls[-1][1][2], value)
            for value in [-3.4, .1]:
                count = len(adapter.calls)
                with self.assertRaises(Rejected):
                    controller.submit({"type": "gripper", "arm": side, "position": value}, "test")
                self.assertEqual(len(adapter.calls), count)

    def test_missing_chassis_and_tcp_limits_still_block_execution(self):
        self.assertGreater(self.config["body"]["max_linear_m_s"], 0)
        self.assertGreater(self.config["body"]["max_angular_rad_s"], 0)
        controller, adapter = self.controller()
        controller.config["body"]["chassis_verified"] = False
        for side in ("left", "right"):
            self.assertIsNone(self.config["arms"][side]["tcp_limits_mm"])
            with self.assertRaises(Rejected):
                controller.submit({"type": "arm.tcp", "arm": side,
                                   "position_mm": [0.] * 3, "orientation_deg": [0.] * 3}, "test")
        with self.assertRaises(Rejected):
            controller.submit({"type": "chassis", "direction": "forward", "speed": .05,
                               "turn_speed": .1, "sequence": 1}, "test")
        self.assertEqual(adapter.calls, [])


if __name__ == "__main__":
    unittest.main()
