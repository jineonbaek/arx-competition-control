"""Exercise the real scheduler/placement handoff with inert robot and task IO."""
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from competition_task import CompetitionTask, Interrupted
from navigation_placement import NavigationPlacement
from navigation_session import NavigationSession
from t2_test_task import T2TestTask
from test_control import Controller, FakeAdapter, configured
from test_competition_task import Module


class PlacementTokenHandoffTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.adapter = FakeAdapter()
        self.ctrl = Controller(configured(), self.adapter)
        self.lease = self.ctrl.reserve_competition(self.ctrl.command_epoch)
        self.events = []
        client = SimpleNamespace(event=lambda visit, event, **kw: self.events.append(event))
        self.nav = NavigationSession(self.ctrl, None, client)
        self.nav.active = True
        self.nav.visit = dict(point_id='point_2', mission_id='mission', visit_id='visit')
        self.nav.mission = 'mission'
        self.s = dict(run_id='run', token=None, competition_token=self.lease,
                      cancel=threading.Event(), records={})
        self.nav.context = dict(run_id='run', cancelled=self.s['cancel'])
        self.place = NavigationPlacement(self.ctrl, None, None, self.nav,
                                         self.temp.name, lidar_locked=True)

    def task(self, task_type=CompetitionTask):
        task = task_type(self.ctrl, self.temp.name, navigation=self.nav)
        # Skip only disk reporting; ownership and handoff implementations stay real.
        save = patch.object(task, '_save')
        save.start()
        self.addCleanup(save.stop)
        return task

    def execute_place(self, task):
        return task._execute(self.s, 'place', self.place, dict(request_id='run:place'))

    def test_session_reference_does_not_skip_arm_lease_for_either_scheduler(self):
        for task_type in (CompetitionTask, T2TestTask):
            with self.subTest(scheduler=task_type.__name__):
                task = self.task(task_type)
                task._mode(self.s, True)
                self.assertIsNone(self.s['token'])
                self.place.visits.clear()
                self.events.clear()

                def launch(context, phase, **fields):
                    self.assertTrue(self.ctrl.motion_valid(context['token']))
                    self.assertFalse(self.lease.navigation)
                    self.assertEqual(self.events, ['accept'])
                    return dict(status='completed')

                # Stop at the actuator worker boundary; no physical commands.
                with patch.object(self.place, '_launch', side_effect=launch) as worker:
                    result = self.execute_place(task)
                    self.assertEqual(result['status'], 'completed')
                    self.assertEqual(worker.call_count, 1)
                self.assertTrue(self.ctrl.snapshot()['motion_reserved'])
                task._mode(self.s, True)
                self.assertFalse(self.ctrl.snapshot()['motion_reserved'])
        self.assertEqual(self.adapter.calls, [])

    def test_explicit_navigation_stage_still_releases_arm_lease(self):
        task = self.task()
        task._mode(self.s, False)
        previous = self.s['token']
        stage = Module([], 'navigate_exit')
        stage.navigation = True
        task._execute(self.s, 'navigate_exit', stage, dict(request_id='run:exit'))
        self.assertIsNone(self.s['token'])
        self.assertFalse(self.ctrl.motion_valid(previous))
        self.assertTrue(self.lease.navigation)
        self.assertEqual(self.events, [])

    def test_cancelled_handoff_never_accepts_or_launches_placement(self):
        task = self.task()
        self.s['cancel'].set()
        with patch.object(self.place, '_launch') as worker:
            with self.assertRaises(Interrupted):
                self.execute_place(task)
            worker.assert_not_called()
        self.assertEqual(self.events, [])

    def test_interrupted_lease_is_not_replaced_to_bypass_stop(self):
        task = self.task()
        task._mode(self.s, False)
        token = self.s['token']
        self.ctrl.interrupt_motion(token)
        with patch.object(self.place, '_launch') as worker:
            with self.assertRaises(Interrupted):
                self.execute_place(task)
            worker.assert_not_called()
        self.assertIs(self.s['token'], token)
        self.assertIs(self.ctrl.motion_owner, token)
        self.assertFalse(self.ctrl.motion_valid(token))
        self.assertEqual(self.events, [])


if __name__ == '__main__':
    unittest.main()
