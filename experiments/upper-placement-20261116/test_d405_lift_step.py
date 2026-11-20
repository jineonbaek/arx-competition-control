import unittest

from d405_lift_step import lift_goal, validate_state


def state(height=7.4, epoch=12, gripper=-1.73):
    return {
        "execution_enabled": True,
        "motion_reserved": False,
        "competition_reserved": False,
        "chassis_active": False,
        "command_epoch": epoch,
        "arms": {
            "left": {"fresh": True, "joints": [0.0] * 6},
            "right": {
                "fresh": True,
                "joints": [0.9, 1.2, 0.2, 0.9, 0.9, -1.5],
                "gripper": gripper,
            },
        },
        "body": {
            "fresh": True,
            "height": height,
            "waist": 0.0,
            "head_yaw": 0.0,
            "head_pitch": 0.0,
        },
        "limits": {"lift": [0.0, 18.0]},
    }


class LiftGoalTests(unittest.TestCase):
    def test_positive_world_z_step_converts_to_lift_radians(self):
        self.assertAlmostEqual(lift_goal(7.4, 15.0), 7.4 + 0.015 * 41.54)

    def test_step_is_bounded(self):
        with self.assertRaisesRegex(ValueError, "bounded"):
            lift_goal(7.4, 15.01)

    def test_goal_respects_reported_lift_limits(self):
        with self.assertRaisesRegex(ValueError, "limits"):
            lift_goal(17.8, 10.0, limits=(0.0, 18.0))


class GuardTests(unittest.TestCase):
    def test_matching_fresh_idle_state_passes(self):
        before = state()
        validate_state(state(), before, expected_epoch=12)

    def test_gripper_change_is_rejected(self):
        before = state()
        with self.assertRaisesRegex(ValueError, "Clamp"):
            validate_state(state(gripper=-1.60), before, expected_epoch=12)

    def test_concurrent_command_is_rejected(self):
        before = state()
        with self.assertRaisesRegex(ValueError, "Concurrent"):
            validate_state(state(epoch=13), before, expected_epoch=12)

    def test_right_arm_motion_is_rejected(self):
        before = state()
        moved = state()
        moved["arms"]["right"]["joints"][2] += 0.02
        with self.assertRaisesRegex(ValueError, "arm changed"):
            validate_state(moved, before, expected_epoch=12)


if __name__ == "__main__":
    unittest.main()
