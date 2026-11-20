"""Failure diagnostics reach the existing durable receipt without changing decisions."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

from agent.tests import test_pick_monitor
from agent.pick_backend import PickExecutionBackend


class PickFeedbackLogTests(unittest.TestCase):
    def setUp(self):
        self.fixture = test_pick_monitor.PickMonitorTests()
        self.fixture.setUp()

    def test_closing_drift_names_phase_and_error_and_is_persisted(self):
        f = self.fixture
        monitor = f.make()
        f.arrive(monitor)
        monitor.start_closing(f.state(10.32), 10.32, ready=True)
        state = f.state(10.42)
        state['arms']['right']['joints'][1] += .1
        receipt = monitor.update(state, 10.42, ready=True)
        self.assertEqual(receipt['status'], 'feedback_invalid')
        self.assertTrue(receipt['stop_required'])
        detail = receipt['failure_detail']
        for text in ('closing', 'TCP到位后偏移', 'XYZ=', 'rotvec=', 'limits='):
            self.assertIn(text, detail)
        with tempfile.TemporaryDirectory() as directory:
            journal = Path(directory) / 'receipt.json'
            args = {'plan_id': 'p', 'plan': {'plan_id': 'p', 'arm': 'right'}}
            step = SimpleNamespace(start=lambda: receipt, cancel=lambda: receipt)
            backend = PickExecutionBackend(step, request_id='test', arguments=args,
                journal=journal, cancelled=lambda: False, timeout_s=1.)
            backend.invoke('execute_pick', args, 'test')
            self.assertTrue(backend.wait(2.))
            self.assertEqual(json.loads(journal.read_text())['receipt']['data']['failure_detail'], detail)

    def test_feedback_age_and_pairing_values_are_reported(self):
        f = self.fixture
        for mode in ('stale', 'skew'):
            with self.subTest(mode=mode):
                monitor = f.make()
                state = f.state(10.4)
                if mode == 'stale':
                    now = 10.6
                else:
                    now = 10.4
                    state['body']['received'] = 10.3
                receipt = monitor.update(state, now, ready=True)
                self.assertEqual(receipt['status'], 'feedback_invalid')
                for text in ('arm_age=', 'lift_age=', 'skew=', 'max_age=', 'max_skew='):
                    self.assertIn(text, receipt['failure_detail'])

    def test_lift_actual_expected_and_existing_limit_are_reported(self):
        f = self.fixture
        monitor = f.make()
        state = f.state(10.1)
        state['body']['height'] += .1
        receipt = monitor.update(state, 10.1, ready=True)
        self.assertEqual(receipt['status'], 'feedback_invalid')
        for text in ('actual=12.10000', 'expected=12.00000', 'limit=0.02'):
            self.assertIn(text, receipt['failure_detail'])

    def test_arrival_stall_reports_current_tcp_error_and_existing_tolerances(self):
        f = self.fixture
        monitor = f.make()
        monitor.target[2, 3] += .013
        for t in (10.01, 10.11, 10.22, 10.42, 10.62, 10.82, 11.02):
            receipt = monitor.update(f.state(t), t, ready=True)
        self.assertEqual(receipt['status'], 'stalled')
        for text in ('预抓/插入到位停止[stalled]', 'XYZ=', '-13.0', 'rotvec=', 'limits=', 'stall_s='):
            self.assertIn(text, receipt['failure_detail'])
        self.assertEqual(monitor.update(f.state(11.1), 11.1, ready=True), receipt)

    def test_opening_stall_keeps_diagnostic_and_existing_stop(self):
        from agent.gripper_open import OpenThenApproachStep
        import numpy as np
        f = self.fixture
        controller = SimpleNamespace(clock=lambda: 10., config={'arms': {'right': {
            'gripper_open_raw': -3.14, 'gripper_limits_raw': [-3.14, 0.]}}})
        child = SimpleNamespace(controller=controller, token='owned', monitor=f.make(),
            preflight=lambda *a: True, cancel=lambda: {'status': 'cancelled', 'stop_required': True})
        step = OpenThenApproachStep(child, f.q)
        step.arrival.previous = np.array([2.7])
        receipt = step._abort('stalled')
        self.assertEqual(receipt['status'], 'stalled')
        self.assertTrue(receipt['stop_required'])
        for text in ('夹爪张开阶段停止[stalled]', '[2.7]', 'minimum=[2.9]'):
            self.assertIn(text, receipt['failure_detail'])
        self.assertEqual(step._abort('timeout'), receipt)

    def test_valid_feedback_has_no_failure_detail(self):
        f = self.fixture
        monitor = f.make()
        receipt = monitor.update(f.state(10.1), 10.1, ready=True)
        self.assertEqual(receipt['status'], 'inserting')
        self.assertIsNone(receipt['failure_detail'])


if __name__ == '__main__':
    unittest.main()
