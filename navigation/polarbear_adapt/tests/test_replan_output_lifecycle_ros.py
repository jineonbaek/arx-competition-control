"""Actual MPPI recovery loop, private ROS197, inert Controller transport only."""
import json, os, time, unittest
from concurrent.futures import Future
from pathlib import Path
from unittest.mock import patch
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, String
import test_mission_ros as fixtures


class Handle:
    accepted = True
    status = 2
    def __init__(self): self.result = Future()
    def get_result_async(self): return self.result
    def cancel_goal_async(self): self.status = 5; return Future()


class ReplanOutputLifecycle(unittest.TestCase):
    setUpClass = classmethod(lambda cls: rclpy.init())
    tearDownClass = classmethod(lambda cls: rclpy.shutdown())
    setUp = fixtures.MissionROS.setUp
    tearDown = fixtures.MissionROS.tearDown
    refresh = fixtures.MissionROS.refresh
    add = fixtures.MissionROS.add

    def prepare(self, failures, *, outcome='arrival'):
        n = self.n
        self.add('stop', 2.); n.waypoints.start(None); n.seen = n.seq
        self.original_target = n.target.copy()
        self.identity = n.transport.current
        self.messages = []; self.replans = 0; self.sends = 0
        self.reasons = []; self.phase_sends = 0; self.after_replan_sends = 0
        original_capture = n.transport.submit
        def capture(m):
            self.messages.append(dict(m))
            if m['type'] != 'chassis.end_output' and any((m['x'], m['y'], m['omega'])):
                self.sends += 1; self.phase_sends += 1
                if self.replans: self.after_replan_sends += 1
            return original_capture(m)
        n.transport.submit = capture
        self.route = np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
        n.route = self.route.copy(); n.route_time = n.now()
        outer = self
        class Client:
            callback = None
            def wait_for_server(self, **kw): return True
            def send_goal_async(self, request, feedback_callback):
                self.callback = feedback_callback
                self.handle = Handle()
                future = Future(); future.set_result(self.handle); return future
        client = Client(); n.mppi_client = client
        prefix = n.validate_active_prefix
        def audit(pose, odom):
            prefix(pose, odom)  # Keep actual footprint/prefix audit in the test.
            if self.phase_sends and self.replans < len(failures):
                reason = failures[self.replans]; self.reasons.append(reason)
                self.replans += 1; self.phase_sends = 0
                raise ValueError(reason)
        n.validate_active_prefix = audit
        self.n.destroy_timer(self.timer)
        began = time.monotonic(); self.cancelled = False
        def feed():
            self.refresh(); now = time.monotonic()
            if now - began > 12: raise AssertionError('fixture exceeded bounded recovery time')
            if n.recovery is not None:
                if outcome == 'cancel' and not self.cancelled:
                    self.cancelled = True; n.cancel('cancelled'); return
                n.route = self.route.copy(); n.route_time = n.now()
            stamp = n.get_clock().now().to_msg()
            sdk = TwistStamped(); sdk.header.stamp = stamp; sdk.twist.linear.x = .08
            n.mppi_received.update(inputs=(now, Bool(data=True)), sdk=(now,sdk),
                                   status=(now,String(data='preview_ready')))
            if client.callback: client.callback(None)
            if self.replans == len(failures) and self.phase_sends >= 2:
                if outcome == 'terminal': raise ValueError('fixture_terminal_guard')
                if outcome == 'arrival':
                    self.position[:] = [2.,0.,0.]; self.refresh()
                    if not client.handle.result.done():
                        from types import SimpleNamespace
                        client.handle.status = 4
                        client.handle.result.set_result(SimpleNamespace(status=4,
                            result=SimpleNamespace(error_code=0,error_msg='')))
        self.timer = n.create_timer(.015,feed); feed()

    def scoped_run(self):
        with self.n.transport.task_scope(self.identity):
            return self.n.run_goal()

    def check_same_goal(self):
        self.assertEqual(self.n.transport.current, self.identity)
        np.testing.assert_allclose(self.n.target, self.original_target)
        moves = [m for m in self.messages if m['type'] != 'chassis.end_output']
        self.assertTrue(moves)
        self.assertEqual({(m['source_id'],m['task_id'],m['generation']) for m in moves},
                         {(self.identity.source_id,self.identity.task_id,self.identity.generation)})
        self.assertEqual([m['seq'] for m in moves], sorted(m['seq'] for m in moves))
        self.assertTrue(any(not any((m['x'],m['y'],m['omega'])) for m in moves))

    def test_recorded_sweep_then_repeated_replan_keeps_identity_and_goal(self):
        reason = 'map_sweep_blocked recorded_grid'
        record = os.getenv('ARX_REPLAN_RECORD')
        if record:
            lines = Path(record).read_text().splitlines()
            reason = next(s.split('waiting_for_same_goal_replan ',1)[1] for s in lines
                          if 'waiting_for_same_goal_replan map_sweep_blocked ' in s)
        self.prepare([reason, 'odom_sweep_blocked second_observation'])
        self.assertEqual(self.scoped_run(), 'arrived_stopped')
        self.assertEqual(self.replans, 2); self.assertGreater(self.after_replan_sends, 0)
        self.check_same_goal()
        self.assertFalse(any(m['type']=='chassis.end_output' for m in self.messages))
        self.n.end_output(self.identity)
        self.assertEqual(self.messages[-1]['type'], 'chassis.end_output')
        self.assertEqual(sum(m['type']=='chassis.end_output' for m in self.messages), 1)

    def test_input_outage_recovery_keeps_output_identity(self):
        self.prepare(['input outage exceeded .30 seconds'])
        self.assertEqual(self.scoped_run(), 'arrived_stopped')
        self.check_same_goal(); self.assertGreater(self.after_replan_sends, 0)
        self.assertFalse(any(m['type']=='chassis.end_output' for m in self.messages))

    def test_cancel_during_wait_ends_old_output_and_never_resumes(self):
        self.prepare(['map_sweep_blocked fixture'], outcome='cancel')
        with self.assertRaisesRegex(ValueError, 'goal_changed'): self.scoped_run()
        self.assertIsNone(self.n.target); self.assertFalse(self.n.active)
        self.assertEqual(sum(m['type']=='chassis.end_output' for m in self.messages), 1)
        self.assertEqual(self.after_replan_sends, 0)

    def test_unknown_failure_in_serve_parks_until_explicit_cancel(self):
        self.prepare([], outcome='terminal')
        status=self.n.status;parked=[]
        def observe(value):
            status(value)
            if value.startswith('paused_navigation'):
                parked.append(value)
                self.assertIsNotNone(self.n.target)
                self.n.cancel('cancelled')
        self.n.status=observe
        # Only the actual outer serve loop owns terminal cleanup. No SDK/network.
        with patch('goal_drive.rclpy.ok', side_effect=lambda: self.n.target is not None):
            self.n.serve()
        self.assertIsNone(self.n.target); self.assertFalse(self.n.active)
        self.assertEqual(sum(m['type']=='chassis.end_output' for m in self.messages), 1)
        self.assertIsNone(self.n.transport.last_identity)
        self.assertTrue(parked)

    def test_arrival_in_serve_still_ends_output_once(self):
        self.prepare([])
        with patch('goal_drive.rclpy.ok', side_effect=lambda: self.n.target is not None):
            self.n.serve()
        self.assertIsNone(self.n.target); self.assertFalse(self.n.active)
        self.assertEqual(sum(m['type']=='chassis.end_output' for m in self.messages), 1)

    def test_brake_transport_failure_still_attempts_terminal_end(self):
        self.prepare(['map_sweep_blocked fixture'])
        capture = self.n.transport.submit
        rejected = []
        def fail_brake(m):
            if (self.sends and m['type'] != 'chassis.end_output'
                    and not any((m['x'],m['y'],m['omega']))):
                rejected.append(m); raise TimeoutError('fixture brake transport failure')
            return capture(m)
        self.n.transport.submit = fail_brake
        timer=self.n.create_timer(.05,lambda:self.n.cancel('cancelled') if rejected and self.n.target is not None else None)
        try:
            with patch('goal_drive.rclpy.ok', side_effect=lambda: self.n.target is not None):self.n.serve()
        finally:self.n.destroy_timer(timer)
        self.assertTrue(rejected)
        self.assertIsNone(self.n.target)
        self.assertEqual(sum(m['type']=='chassis.end_output' for m in self.messages), 1)
        self.assertIsNone(self.n.transport.last_identity)


if __name__ == '__main__': unittest.main()
