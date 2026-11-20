"""Real hub/route/stop proofs; ROS and actuator edges are inert."""
import copy
import importlib.util
import os
import sys
import tempfile
import types
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission_route import MissionRoute
from task_handoff import HandoffHub


def load_runtime():
    class Message:
        def __init__(self, *a, **kw): self.__dict__.update(kw)
    modules = {}
    for name in ('rclpy', 'rclpy.time', 'rclpy.qos', 'std_msgs', 'std_msgs.msg'):
        modules[name] = types.ModuleType(name)
    modules['rclpy.time'].Time = Message
    modules['rclpy.qos'].QoSProfile = Message
    modules['rclpy.qos'].DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    modules['std_msgs.msg'].Bool = Message
    # recovery's numpy helpers are unused here; real RecoveryWindow needs no numpy.
    if importlib.util.find_spec('numpy') is None:
        modules['numpy'] = types.ModuleType('numpy')
    spec = importlib.util.spec_from_file_location('task_resume_retry_target', ROOT / 'mission_runtime.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules): spec.loader.exec_module(module)
    return module


runtime_module = load_runtime()


class Vector(list):
    def tolist(self): return list(self)
    def copy(self): return Vector(self)


class Driver:
    def __init__(self):
        self.t = 10.; self.ready_file = None; self.target = None; self.arrow = None
        self.stale = []; self.position = Vector([0., 0., 0.])
        self.end_ready = True; self.pending_output_ends = set()
        self.state = dict(execution_enabled=True, body=dict(fresh=True), chassis_active=False,
            nav_sdk_ready=True, capabilities=dict(chassis=False))
        self.advance(0.)
    def advance(self, dt=.1):
        self.t += dt
        self.pose_snapshot = types.SimpleNamespace(header=types.SimpleNamespace(stamp=
            types.SimpleNamespace(sec=int(self.t), nanosec=int((self.t % 1)*1e9))))
        grid = types.SimpleNamespace(inspect=lambda pose: dict(first_cell=None))
        self.grids = {f: (self.pose_snapshot.header.stamp, grid) for f in ('map', 'odom')}
    def now(self): return self.t
    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(self.t*1e9)))
    def create_publisher(self, *args): return types.SimpleNamespace(publish=lambda msg: None)
    def end_output(self):
        self.pending_output_ends.add('original-output')
        if not self.end_ready:return False
        self.pending_output_ends.discard('original-output')
        return True
    def retry_output_ends(self):
        if self.pending_output_ends:return self.end_output()
        return True
    def motion_poses(self): return self.position.copy(), self.position.copy(), self.t
    special_poses = motion_poses
    def validate_motion_proof(self, stamp):
        if stamp != self.t: raise ValueError('expired motion proof')
    def stale_inputs(self): return self.stale
    special_stale_inputs = stale_inputs
    def get_logger(self): return types.SimpleNamespace(warn=lambda *a, **kw: None)


class Waypoints:
    def __init__(self, driver, route):
        self.driver, self.route = driver, route
        self.map_id = route.map_id; self.goal_stamp = 0
        self.dispatched = []; self.attempts = 0; self.failure = None
    def show(self): pass
    def direct_ready(self, pose, after_task=False):
        return self.route.can_enter_direct(pose, after_task=after_task)
    def dispatch(self, point):
        if point is None: return
        self.attempts += 1
        self.goal_stamp = int(self.driver.t*1e9)
        failure, self.failure = self.failure, None
        if failure == 'before': raise OSError('before goal accepted')
        self.driver.arrow = types.SimpleNamespace(header=types.SimpleNamespace(stamp=
            types.SimpleNamespace(sec=self.goal_stamp//10**9, nanosec=self.goal_stamp % 10**9)))
        self.driver.target = list(point['pose'])
        self.dispatched.append(copy.deepcopy(point))
        if failure == 'after': raise OSError('after goal accepted')


class ResumeRetry(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.d = Driver(); self.route = MissionRoute('room')
        self.route.add(dict(type='task', point_id='point_1', pose=[0., 0., 0.]))
        self.route.add(dict(type='via', pose=[1., 0., 0.]))
        self.w = Waypoints(self.d, self.route)
        with patch.dict(os.environ, {'ROS_DOMAIN_ID':'197', 'ARX_TASK_JOURNAL':str(Path(self.tmp.name)/'hub.json')}):
            self.r = runtime_module.TaskRuntime(self.w)
        self.r.console = lambda: copy.deepcopy(self.d.state)
        self.r.hub.clock = lambda: self.d.t
        self.worker = dict(worker_id='battery', instance_id=str(uuid.uuid4()), point_ids=['point_1', 'point_2'])
        self.token = self.r.hub.register(self.worker)['token']
        self.route.start(self.r.hub.available_points()); self.route.finish('arrived_stopped')
    def tearDown(self): self.r.close(); self.tmp.cleanup()
    def complete(self):
        self.r.notify(self.route.points[0])
        task = self.r.hub.active
        for event in ('accept', 'complete'):
            payload = {k:task[k] for k in ('mission_id', 'visit_id', 'point_id')}
            payload.update(request_id=str(uuid.uuid4()), event=event)
            if event == 'complete': payload.update(control_released=True, navigation_ready=True)
            self.r.hub.event(self.token, payload)
        return task['visit_id']
    def ticks(self, count=35):
        for _ in range(count):
            self.d.advance(); self.r.hub.heartbeat(self.token)
            with patch.object(runtime_module.time, 'monotonic', lambda: self.d.t): self.r.tick()
    def test_dispatch_before_acceptance_is_retried_for_same_next_point(self):
        visit = self.complete(); self.w.failure = 'before'; self.ticks()
        self.assertEqual(self.r.hub.active['visit_id'], visit)
        self.assertEqual(self.route.index, 1)
        self.assertEqual(len(self.w.dispatched), 1)
        self.assertEqual(self.w.dispatched[0]['pose'], [1., 0., 0.])
    def test_exception_after_goal_acceptance_never_accepts_goal_twice(self):
        self.complete(); self.w.failure = 'after'; self.ticks()
        self.assertEqual(self.w.attempts, 1)
        self.assertEqual(len(self.w.dispatched), 1)
        self.assertEqual(self.route.index, 1)
    def test_release_mutated_then_raised_does_not_advance_index_again(self):
        self.complete(); release = self.route.release_task; calls = []
        def fail_after_release(*a, **kw):
            calls.append(True); release(*a, **kw)
            raise OSError('after route index advanced')
        self.route.release_task = fail_after_release
        self.ticks()
        self.assertEqual(len(calls), 1)
        self.assertEqual(self.route.index, 1)
        self.assertEqual(len(self.w.dispatched), 1)
    def test_release_failure_before_mutation_retries_once_conditions_return(self):
        self.complete(); release = self.route.release_task; calls = []
        def fail_once(*a, **kw):
            calls.append(True)
            if len(calls) == 1: raise OSError('release unavailable')
            return release(*a, **kw)
        self.route.release_task = fail_once
        self.ticks()
        self.assertEqual(self.route.index, 1)
        self.assertEqual(len(self.w.dispatched), 1)
    def test_expired_same_worker_notification_waits_and_creates_one_visit(self):
        self.d.advance(2.1)
        self.r.notify(self.route.points[0])
        self.assertTrue(self.route.running)
        self.assertEqual(self.route.state, 'waiting_task')
        self.assertIsNone(self.r.hub.active)
        self.r.hub.register(self.worker); self.ticks()
        self.assertEqual(self.r.hub.active['status'], 'notified')
        self.assertEqual(sum(e['event']=='notified' for e in self.r.hub.events), 1)
        self.assertEqual(self.w.dispatched, [])
    def test_replaced_worker_never_gets_delayed_original_notification(self):
        self.d.advance(2.1)
        self.r.notify(self.route.points[0])
        self.worker['instance_id'] = str(uuid.uuid4())
        self.token = self.r.hub.register(self.worker)['token']; self.ticks()
        self.assertIsNone(self.r.hub.active)
    def test_unready_console_prevents_delayed_notification(self):
        self.d.advance(2.1); self.r.notify(self.route.points[0]); self.r.hub.register(self.worker)
        self.d.state['body']['fresh'] = False; self.ticks()
        self.assertIsNone(self.r.hub.active)
        self.d.state['body']['fresh'] = True; self.ticks()
        self.assertEqual(self.r.hub.active['status'], 'notified')
    def test_delayed_notification_cannot_claim_arrival_after_chassis_moved(self):
        self.d.advance(2.1); self.r.notify(self.route.points[0]); self.r.hub.register(self.worker)
        self.d.position[0] = .3; self.ticks()
        self.assertIsNone(self.r.hub.active)
    def test_stale_inputs_hold_unaccepted_dispatch_until_fresh_again(self):
        self.complete(); self.w.failure = 'before'
        while self.w.attempts == 0: self.ticks(1)
        self.d.stale = ['pose']; self.ticks()
        self.assertEqual(self.w.attempts, 1)
        self.d.stale = []; self.ticks()
        self.assertEqual(len(self.w.dispatched), 1)
    def test_cancelled_route_does_not_retry_unaccepted_dispatch(self):
        self.complete(); self.w.failure = 'before'
        while self.w.attempts == 0: self.ticks(1)
        self.route.cancel(); self.ticks()
        self.assertEqual(self.w.attempts, 1)
        self.assertEqual(self.w.dispatched, [])
    def test_new_visit_does_not_reuse_old_accepted_dispatch_intent(self):
        self.route.points[1].update(type='task', point_id='point_2')
        self.route.points.append(dict(id='exit', type='via', pose=[2., 0., 0.], pass_radius_m=.1))
        self.complete(); self.w.failure = 'after'
        while self.w.attempts == 0: self.ticks(1)
        self.d.position = Vector(self.route.points[1]['pose']); self.d.advance(.1)
        self.route.finish('arrived_stopped'); self.r.notify(self.route.points[1])
        task = self.r.hub.active
        for event in ('accept', 'complete'):
            payload = {k:task[k] for k in ('mission_id', 'visit_id', 'point_id')}
            payload.update(request_id=str(uuid.uuid4()), event=event)
            if event == 'complete':payload.update(control_released=True, navigation_ready=True)
            self.r.hub.event(self.token, payload)
        self.ticks()
        self.assertEqual(self.route.index, 2)
        self.assertEqual(len(self.w.dispatched), 2)
        self.assertEqual(self.w.dispatched[-1]['id'], 'exit')
    def test_worker_replacement_racing_failed_begin_cannot_take_pending_visit(self):
        self.d.advance(2.1); begin = self.r.hub.begin
        def replace_on_error(value):
            try:return begin(value)
            except ValueError:
                self.worker = dict(self.worker, instance_id=str(uuid.uuid4()))
                self.token = self.r.hub.register(self.worker)['token']
                raise
        self.r.hub.begin = replace_on_error
        self.r.notify(self.route.points[0]); self.r.hub.begin = begin
        self.ticks()
        self.assertIsNone(self.r.hub.active)
    def test_unconfirmed_output_end_defers_even_live_worker_notification(self):
        self.d.end_ready = False
        self.r.notify(self.route.points[0])
        self.assertIsNone(self.r.hub.active)
        self.assertIsNotNone(self.r.pending_notification)
        self.assertTrue(self.route.running)
        self.assertEqual(self.route.state, 'waiting_task')
        self.ticks()
        self.assertIsNone(self.r.hub.active)
        self.d.end_ready = True; self.ticks(1)
        self.assertIsNone(self.r.hub.active)  # Require fresh post-drain stop proof.
        self.ticks()
        self.assertEqual(self.r.hub.active['status'], 'notified')
        self.assertEqual(sum(e['event']=='notified' for e in self.r.hub.events), 1)
        self.assertEqual(self.w.dispatched, [])
    def test_unconfirmed_output_end_prevents_ack_and_first_dispatch(self):
        self.complete(); self.d.end_ready = False; self.d.end_output(); self.ticks()
        self.assertEqual(self.r.hub.active['status'], 'resume_pending')
        self.assertEqual(self.route.index, 0)
        self.assertEqual(self.w.dispatched, [])
        self.d.end_ready = True; self.ticks()
        self.assertEqual(self.r.hub.active['status'], 'acknowledged')
        self.assertEqual(self.route.index, 1)
        self.assertEqual(len(self.w.dispatched), 1)
    def test_unconfirmed_output_end_prevents_retry_dispatch_after_ack(self):
        self.complete(); self.w.failure = 'before'
        while self.w.attempts == 0:self.ticks(1)
        self.d.end_ready = False; self.d.end_output(); self.ticks()
        self.assertEqual(self.w.attempts, 1)
        self.assertEqual(self.route.index, 1)
        self.d.end_ready = True; self.ticks()
        self.assertEqual(len(self.w.dispatched), 1)
        self.assertEqual(self.route.index, 1)
    def test_stale_initial_motion_batch_recovers_one_real_arrival_visit(self):
        poses = self.d.motion_poses
        def stale():raise ValueError('motion_pose_stale_or_invalid')
        self.d.motion_poses = stale
        self.r.notify(self.route.points[0])
        self.assertTrue(self.route.running)
        self.assertEqual(self.route.state, 'waiting_task')
        self.assertIsNone(self.r.hub.active)
        self.assertIsNotNone(self.r.pending_notification)
        self.ticks(3)
        self.d.motion_poses = poses; self.d.position[0] = .01; self.ticks()
        self.assertEqual(self.r.hub.active['status'], 'notified')
        self.assertEqual(self.r.hub.active['arrival_pose'], [.01, 0., 0.])
        self.assertEqual(sum(e['event']=='notified' for e in self.r.hub.events), 1)
    def test_failed_initial_proof_is_not_an_arrival_anchor(self):
        validate = self.d.validate_motion_proof
        self.d.position[0] = .3
        def changed(source):raise ValueError('motion_pose_changed_during_proof')
        self.d.validate_motion_proof = changed
        self.r.notify(self.route.points[0])
        self.assertIsNone(self.r.hub.active)
        self.assertIsNotNone(self.r.pending_notification)
        self.d.validate_motion_proof = validate; self.ticks()
        self.assertIsNone(self.r.hub.active)  # Stable, but outside the original goal.
        self.d.position[0] = .01; self.ticks()
        self.assertEqual(self.r.hub.active['arrival_pose'], [.01, 0., 0.])
        self.assertEqual(sum(e['event']=='notified' for e in self.r.hub.events), 1)
    def test_unknown_initial_motion_error_is_not_automatically_retried(self):
        def unknown():raise RuntimeError('unexpected pose contract violation')
        self.d.motion_poses = unknown
        with self.assertRaisesRegex(RuntimeError, 'unexpected pose contract violation'):
            self.r.notify(self.route.points[0])
        self.assertIsNone(self.r.pending_notification)


if __name__ == '__main__': unittest.main()
