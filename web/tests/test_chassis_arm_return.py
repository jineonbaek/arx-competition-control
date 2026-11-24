"""Run the real release/return executor with inert joint feedback, never hardware."""
from types import SimpleNamespace
from unittest.mock import patch
import unittest
import numpy as np

from docking_motion import DockingMotion
from test_chassis_placement import FinalizeTests


class ArmReturnTests(unittest.TestCase):
    def fixture(self, problem=None):
        finalizer, context, events = FinalizeTests().fixture()
        del finalizer._arm_return  # Exercise the production return and its real motion guard.
        ctrl = finalizer.ctrl
        now, epoch, joints, sent = [0.], [0], np.array([.01, 0., 0., 0., 0., 0.]), []
        original_snapshot = ctrl.snapshot
        prepared = finalizer.placement.runs['run:place']['placement']
        prepared['chassis_return_route_rad'] = [[0.]*6, joints.tolist()]
        def snapshot():
            state = original_snapshot()
            state['arms']['right'].update(joints=joints.tolist(), received=now[0])
            if 'retreat' in events and problem == 'feedback':
                state['arms']['right']['fresh'] = False
            return state
        def driver(side):
            return dict(driver_instance='other' if 'retreat' in events and problem == 'driver' else 'driver',
                        ready=True, enabled=True, command_epoch=epoch[0])
        def sleep(seconds):
            now[0] += seconds
        def start(side, plan, token, *, preflight, expected_driver, require_lift_settle):
            self.assertIs(preflight(snapshot(), now[0]), True)
            self.assertEqual(expected_driver, dict(driver_instance='driver', command_epoch=epoch[0]))
            sent.append(plan)
            old_epoch = epoch[0]
            epoch[0] += 1
            joints[:] = plan['joint_waypoints_rad'][-1]
            return dict(driver_instance='driver', command_epoch=old_epoch)
        def poll(token, *, preflight):
            self.assertIs(preflight(snapshot(), now[0]), True)
            return dict(status='reference_complete')
        def transform(q, height):
            result = np.eye(4)
            result[:3, 3] = q[:3]
            return result
        ctrl.snapshot, ctrl.clock, ctrl.adapter.trajectory_status = snapshot, lambda: now[0], driver
        ctrl.motion_valid = lambda token: not ('retreat' in events and problem == 'owner')
        ctrl.start_trajectory, ctrl.poll_trajectory = start, poll
        ctrl.models = {'right': SimpleNamespace(reach=1., wrist_transform=transform)}
        finalizer.backend.tcp = np.eye(4)
        finalizer.backend.limits = dict(joint_speed_deg_s=[15.]*6, joint_acceleration_deg_s2=[18.]*6,
                                        tcp_speed_mm_s=60., tcp_acceleration_mm_s2=120.)
        finalizer.backend.timeline = SimpleNamespace(plan=lambda seconds: None, complete=lambda: None, snapshot=lambda: {})
        return finalizer, context, events, sent, lambda *a, **kw: DockingMotion(*a, sleep=sleep, **kw)

    def run_return(self, problem=None):
        finalizer, context, events, sent, motion = self.fixture(problem)
        with patch('docking_motion.DockingMotion', side_effect=motion):
            finalizer.start(context)
            finalizer.runs[context['request_id']]['thread'].join(2.)
        self.assertFalse(finalizer.runs[context['request_id']]['thread'].is_alive())
        return finalizer.inspect(context), events, sent

    def test_valid_return_reaches_initial_joints_after_release_and_retreat(self):
        result, events, sent = self.run_return()
        self.assertEqual(result['status'], 'completed', result)
        self.assertTrue(result['returned'])
        self.assertEqual(len(sent), 1)
        np.testing.assert_allclose(sent[-1]['joint_waypoints_rad'][-1], np.zeros(6))
        self.assertLess(events.index('release'), events.index('retreat'))

    def test_invalid_owner_or_feedback_still_prevents_arm_dispatch(self):
        for problem in ('owner', 'feedback', 'driver'):
            with self.subTest(problem=problem):
                result, _, sent = self.run_return(problem)
                self.assertEqual(result['status'], 'failed')
                self.assertTrue(result['released'])
                self.assertFalse(result['returned'])
                self.assertEqual(sent, [])


if __name__ == '__main__':
    unittest.main()
