"""Only initial pregrasp acceptance changes; insertion/correction keep their policy."""
import copy
import unittest

import numpy as np

import test_battery_step_preparation as fixtures


class PregraspToleranceTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.PreparationTests()
        self.f.setUp()
        self.f.policies['pick']['arm_policy']['tolerance'] = [5., 5., 5., 1.5, 1.5, 1.5]

    def test_stationary_and_moving_pregrasp_use_10mm_3deg_without_mutating_policy(self):
        f = self.f
        original = copy.deepcopy(f.policies)
        for moving in (False, True):
            if moving:
                f.plan['start_joints_rad'][0] -= .001
                f.plan['approach']['start_rad'] = f.plan['start_joints_rad'][:]
            step = f.prepare().approach
            np.testing.assert_array_equal(step.monitor.arm.tolerance, [5., 5., 5., 1.5, 1.5, 1.5])
            step.monitor.begin_settling(f.c.clock())
            np.testing.assert_array_equal(step.monitor.arm.tolerance, [10., 10., 10., 3., 3., 3.])
            self.assertEqual(f.policies, original)
        self.assertEqual(f.f.adapter.calls, [])

    def test_final_insertion_retains_5mm_1point5deg(self):
        step = self.f.prepare('execute_pick')
        np.testing.assert_array_equal(step.monitor.arm.tolerance, [5., 5., 5., 1.5, 1.5, 1.5])
        self.assertEqual(self.f.f.adapter.calls, [])

    def test_visual_correction_retains_original_tolerance(self):
        f = self.f
        end = np.asarray(f.plan['joint_waypoints_rad'][-1])
        target = f.c.models['right'].wrist_transform(end, 12.) @ f.f.tcp
        f.plan['correction'] = dict(joint_waypoints_rad=f.plan['joint_waypoints_rad'],
            target_base_from_tool=target.tolist(), column_checked=True,
            timing={'status': 'timed_preview'})
        f.plan['refinement'] = dict(status='correction_required',
            correction_target_base_from_tool=target.tolist())
        step = f.prepare()
        np.testing.assert_array_equal(step.monitor.arm.tolerance, [5., 5., 5., 1.5, 1.5, 1.5])
        self.assertEqual(f.f.adapter.calls, [])


if __name__ == '__main__':
    unittest.main()
