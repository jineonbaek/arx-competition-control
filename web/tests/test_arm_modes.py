import copy
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

from test_control import FakeAdapter, configured
from control import ARM_MODES, Controller, Rejected
from ros_adapter import RosAdapter


class ArmModeTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.config = configured()
        for arm in self.config["arms"].values():
            arm["verified_modes"] = [0, 1, 2, 3]
        self.adapter = FakeAdapter()
        self.controller = Controller(self.config, self.adapter, lambda: self.now)
        for side, value in [("left", .1), ("right", .2)]:
            self.controller.update_arm(side, [value] * 6, -1.2, [value] * 6)

    def test_all_four_modes_target_selected_arm_and_preserve_complete_feedback(self):
        for side in ("left", "right"):
            state = copy.deepcopy(self.controller.arms[side])
            for mode in range(4):
                with self.subTest(side=side, mode=mode):
                    result = self.controller.submit({"type": "arm.mode", "arm": side, "mode": mode}, "test")
                    self.assertEqual(self.adapter.calls[-1],
                                     ("arm", (side, state["joints"], state["gripper"], state["tcp"], mode)))
                    self.assertIn(str(mode), result["message"])
                    self.assertIn(ARM_MODES[mode], result["message"])
                    self.assertIn("unconfirmed", result["mode_status"])
                    self.assertEqual(self.controller.arms[side], state)
        self.assertEqual(len(self.adapter.calls), 8)
        self.assertNotIn("mode", self.controller.snapshot()["arms"]["left"])

    def test_mode_requires_integer_0_to_3_and_cannot_inject_targets(self):
        for mode in [True, False, 1., "2", None, -1, 4, 5, [], {}]:
            with self.subTest(mode=mode), self.assertRaises(Rejected):
                self.controller.submit({"type": "arm.mode", "arm": "left", "mode": mode}, "test")
        for extra in [{"joints": [0.] * 6}, {"position": 0}, {"end_pos": [0.] * 6}]:
            with self.assertRaises(Rejected):
                self.controller.submit({"type": "arm.mode", "arm": "left", "mode": 2, **extra}, "test")
        self.assertEqual(self.adapter.calls, [])

    def test_observation_and_each_existing_safety_gate_block_modes(self):
        mutations = [
            lambda c: c.config.update(execution_enabled=False),
            lambda c: c.config["arms"]["left"].update(verified=False),
            lambda c: c.config["arms"]["left"].update(verified_modes=[]),
            lambda c: c.config["arms"]["left"].update(joint_limits_deg=None),
            lambda c: c.config["arms"]["left"].update(gripper_limits_raw=None),
            lambda c: c.config["arms"]["left"].update(max_delta_deg=None),
            lambda c: c.arms.update(left=None),
        ]
        for mutate in mutations:
            controller = Controller(self.config, self.adapter, lambda: self.now)
            controller.update_arm("left", [.1] * 6, -1.2, [.1] * 6)
            mutate(controller)
            for mode in range(4):
                with self.assertRaises(Rejected):
                    controller.submit({"type": "arm.mode", "arm": "left", "mode": mode}, "test")
        self.now += 2
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "arm.mode", "arm": "left", "mode": 2}, "test")
        self.now = 100.
        self.adapter.has_subscriber = lambda topic, kind: False
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "arm.mode", "arm": "left", "mode": 2}, "test")
        self.assertEqual(self.adapter.calls, [])

    def test_per_mode_verification_and_capabilities_are_separate_for_each_arm(self):
        self.controller.config["arms"]["left"]["verified_modes"] = [2]
        self.controller.config["arms"]["right"]["verified_modes"] = [3]
        caps = self.controller.snapshot()["capabilities"]
        self.assertEqual(caps["left:arm.mode.2"], "")
        self.assertTrue(caps["left:arm.mode.3"])
        self.assertEqual(caps["right:arm.mode.3"], "")
        self.assertTrue(caps["right:arm.mode.2"])
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "arm.mode", "arm": "left", "mode": 1}, "test")
        self.controller.config["arms"]["left"]["verified_modes"] = [True]
        with self.assertRaises(Rejected):
            self.controller.submit({"type": "arm.mode", "arm": "left", "mode": 1}, "test")
        self.assertEqual(self.adapter.calls, [])

    def test_actual_configuration_enables_documented_modes_with_driver_readiness(self):
        root = Path(__file__).resolve().parents[1]
        config = json.loads((root / "config/control.json").read_text())
        self.assertTrue(all(a["verified"] for a in config["arms"].values()))
        for arm in config["arms"].values():
            self.assertEqual(arm["verified_modes"], [0, 1, 2, 3])
            self.assertTrue(arm["ready_topic"])
            self.assertTrue(arm["stop_service"])

    def test_existing_ros_adapter_serializes_full_mode_request_without_ros_or_can(self):
        adapter = RosAdapter(self.config)
        adapter.RobotCmd = SimpleNamespace
        published = []
        adapter._publisher = lambda topic, kind: SimpleNamespace(
            publish=lambda msg: published.append((topic, msg)))
        controller = Controller(self.config, adapter, lambda: self.now)
        adapter.has_subscriber = lambda topic, kind: True
        adapter.status = lambda: {"connected": True, "error": ""}
        controller.update_arm("right", [.2] * 6, -1.2, [.3] * 6)
        for mode in range(4):
            controller.submit({"type": "arm.mode", "arm": "right", "mode": mode}, "test")
            topic, msg = published[-1]
            self.assertEqual(topic, self.config["arms"]["right"]["command_topic"])
            self.assertEqual(msg.mode, mode)
            self.assertEqual(msg.joint_pos, [.2] * 6)
            self.assertEqual(msg.end_pos, [.3] * 6)
            self.assertEqual(msg.gripper, -1.2)


if __name__ == "__main__":
    unittest.main()
