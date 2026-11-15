"""Prepared in-process right-arm transport seam; no ROS/SDK/CAN construction.

No physical approval provider or server route is installed by this module.
OfflineCabinetRun deliberately continues to accept SimulationExecutor only.
PhysicalApproval is a trusted process-internal provider contract, not a browser
payload, a cryptographic certificate, or a replacement for physical validation.
"""
from dataclasses import asdict, dataclass
import copy
import hashlib
import inspect
import json
import math
from pathlib import Path
from numbers import Real
import re
import threading

from .cabinet_gates import REQUIRED


class BridgeRejected(ValueError):
    pass


def finite(value):
    try:
        return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)
    except (ValueError, OverflowError):
        return False


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class BridgePolicy:
    joint_tolerance_rad: float
    gripper_tolerance_rad: float
    lift_tolerance_rad: float
    rest_speed_rad_s: float
    settle_s: float
    max_age_s: float
    max_poll_gap_s: float
    timeout_s: float

    def __post_init__(self):
        values = asdict(self)
        if (not all(finite(v) and v > 0 for v in values.values())
                or self.settle_s < .1 or self.max_age_s > .5
                or self.max_poll_gap_s > .25 or self.timeout_s > 300
                or self.settle_s >= self.timeout_s):
            raise BridgeRejected('Explicit finite bounded arrival/stop policy required')


@dataclass(frozen=True)
class PhysicalApproval:
    request_hash: str
    issued_at: float
    expires_at: float
    verified_gates: frozenset
    scope: str
    physical_stop_verified: bool
    clearance_mm: float
    error_upper_mm: float
    reserve_mm: float


class ControllerBridge:
    """One right-arm segment per object; cancellation and unknown results latch.

The provider must independently evaluate actual G01–G20 evidence, including held
pose, normal/stopping sweep, calibration, precision and hardware stop behavior.
There is intentionally no production provider here. Fake-provider tests exercise
the seam only. This class never changes gripper, lift or chassis targets.
"""
    PHASES = frozenset(('outside_align', 'insert', 'retreat'))
    TERMINAL = frozenset(('completed', 'cancelled'))

    def __init__(self, controller, policy, *, physical_provider=None):
        if type(policy) is not BridgePolicy:
            raise BridgeRejected('BridgePolicy required')
        self.controller, self.policy, self.provider = controller, policy, physical_provider
        self.lock = threading.RLock()
        self.stop_lock = threading.Lock()
        self.stop_dispatch_lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.stop_inflight = False
        self.cancel_generation = 0
        self.stop_request_time = None
        if 'expected_driver' not in inspect.signature(controller.start_trajectory).parameters:
            raise BridgeRejected('Controller lacks approved driver-epoch dispatch contract')
        self.status = 'idle'
        self.reason = ''
        self.token = self.handle = self.request = self.frozen = None
        self.outcome_unknown = False
        self.stop_sent = False
        self.stop_error = ''
        self.rest = []
        self.rest_after = None
        self.start_time = self.last_poll = None
        self.sources = {}
        for item in (type(controller), type(controller.adapter)):
            try:
                p = Path(inspect.getfile(item)).resolve()
                self.sources[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
            except (TypeError, OSError) as error:
                raise BridgeRejected('Controller source identity unavailable') from error

    def snapshot(self):
        return dict(status=self.status, completed=self.status == 'completed',
                    outcome_unknown=self.outcome_unknown, reason=self.reason,
                    stop_requested=self.stop_sent, stop_error=self.stop_error,
                    cancellation_requested=self.cancel_event.is_set(),
                    ownership_retained=self.token is not None,
                    controller_dispatch_attempted=self.start_time is not None,
                    production_provider_installed=callable(self.provider))

    def _sources_valid(self):
        try:
            return all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h
                       for p, h in self.sources.items())
        except OSError:
            return False

    @staticmethod
    def _vector(values, length):
        return (isinstance(values, list) and len(values) == length
                and all(finite(v) for v in values))

    def _feedback(self, state, now):
        return measured_feedback(state, now, self.policy)

    def _fixed_components(self, values):
        if self.frozen is None:
            return True
        return (abs(values[6]-self.frozen[6]) <= self.policy.gripper_tolerance_rad
                and abs(values[7]-self.frozen[7]) <= self.policy.lift_tolerance_rad
                and all(abs(a-b) <= self.policy.joint_tolerance_rad
                        for a, b in zip(values[8:], self.frozen[8:])))

    def _admission_context(self, state, now):
        if self.cancel_event.is_set():
            raise BridgeRejected('Cancellation is latched')
        if self.start_time is not None and (
                not finite(now) or now < self.start_time or now < self.last_poll
                or now-self.start_time >= self.policy.timeout_s
                or now-self.last_poll > self.policy.max_poll_gap_s):
            raise BridgeRejected('Task deadline or poll lease expired during admission')
        if not self._sources_valid() or not callable(self.provider):
            raise BridgeRejected('Validated physical provider/source identity missing')
        if digest(self.controller.config) != self.request['controller_config_hash']:
            raise BridgeRejected('Controller configuration changed')
        _, values = self._feedback(state, now)
        if not self._fixed_components(values):
            raise BridgeRejected('Grip, lift, left arm or body configuration changed')
        driver = self.controller.adapter.trajectory_status('right')
        if (not isinstance(driver, dict) or not driver.get('ready') or not driver.get('enabled')
                or driver.get('driver_instance') != self.request['driver_instance']):
            raise BridgeRejected('Driver identity/readiness changed')
        if self.handle is None:
            if (driver.get('command_epoch') != self.request['driver_epoch']
                    or driver.get('status') not in ('idle','failed','reference_complete')
                    or driver.get('manual_joint_active',False)
                    or any(abs(a-b)>self.policy.joint_tolerance_rad
                           for a,b in zip(values[:6],self.request['plan']['joint_waypoints_rad'][0]))):
                raise BridgeRejected('Approved driver epoch or actual start changed')
        else:
            matching = (driver.get('command_epoch') == self.handle['command_epoch']+1
                        and driver.get('request_id') == self.handle['request_id']
                        and driver.get('status') in ('running','reference_complete'))
            awaiting = (0 <= now-self.start_time < .3
                        and driver.get('command_epoch') == self.handle['command_epoch']
                        and driver.get('request_id') != self.handle['request_id']
                        and driver.get('status') in ('idle','failed','reference_complete'))
            if not matching and not awaiting:
                raise BridgeRejected('Published trajectory identity/epoch no longer matches')

    def _approve(self, state, now):
        self._admission_context(state, now)
        proof = self.provider(copy.deepcopy(self.request), copy.deepcopy(state), now)
        after = self.controller.clock()
        if not finite(after) or not now <= after <= now+self.policy.max_age_s:
            raise BridgeRejected('Physical evidence provider exceeded admission window')
        self._admission_context(self.controller.snapshot(), after)
        if type(proof) is not PhysicalApproval:
            raise BridgeRejected('PhysicalApproval required; simulator output is not permission')
        if (proof.scope != 'validated_physical' or proof.physical_stop_verified is not True
                or proof.request_hash != self.request['request_hash']
                or type(proof.verified_gates) is not frozenset
                or not set(REQUIRED[self.request['phase']]) <= proof.verified_gates
                or not all(finite(v) for v in (proof.issued_at, proof.expires_at,
                                               proof.clearance_mm, proof.error_upper_mm, proof.reserve_mm))
                or not proof.issued_at <= after < proof.expires_at
                or proof.expires_at-proof.issued_at > self.policy.max_age_s
                or proof.clearance_mm <= 0 or min(proof.error_upper_mm, proof.reserve_mm) < 0
                or proof.error_upper_mm+proof.reserve_mm >= proof.clearance_mm):
            raise BridgeRejected('Missing, stale, mismatched or geometrically unsafe physical approval')
        return True

    def _request(self, phase, plan, binding, state, now):
        if phase not in self.PHASES:
            raise BridgeRejected('Only right-arm outside alignment/insertion/retreat supported')
        keys = {'run_id', 'boot_id', 'generation', 'asset_hash', 'held_identity_hash'}
        if (not isinstance(binding, dict) or set(binding) != keys
                or any(not isinstance(binding[k], str) or not 0 < len(binding[k]) <= 128
                       for k in ('run_id', 'boot_id'))
                or type(binding['generation']) is not int or binding['generation'] < 0
                or any(not isinstance(binding[k], str) or not re.fullmatch('[a-f0-9]{64}', binding[k])
                       for k in ('asset_hash', 'held_identity_hash'))):
            raise BridgeRejected('Run/boot/generation/assets/grasp binding required')
        if (not isinstance(plan, dict)
                or set(plan) != {'interpolation', 'time_from_start_s', 'joint_waypoints_rad'}
                or plan['interpolation'] != 'segment_quintic_scalar'):
            raise BridgeRejected('Exact existing timed-trajectory format required')
        times, points = plan['time_from_start_s'], plan['joint_waypoints_rad']
        if (not isinstance(times, list) or not 2 <= len(times) <= 1024
                or not all(finite(t) for t in times) or times[0] != 0
                or any(b <= a for a, b in zip(times, times[1:]))
                or times[-1]+self.policy.settle_s >= self.policy.timeout_s
                or not isinstance(points, list) or len(points) != len(times)
                or not all(self._vector(q, 6) for q in points)):
            raise BridgeRejected('Invalid timed waypoints')
        _, values = self._feedback(state, now)
        if any(abs(a-b) > self.policy.joint_tolerance_rad for a, b in zip(values[:6], points[0])):
            raise BridgeRejected('Measured start no longer matches trajectory')
        limits = self.controller.config['arms']['right'].get('joint_limits_deg')
        if (not isinstance(limits, list) or len(limits) != 6
                or not all(self._vector(pair, 2) and pair[0] < pair[1] for pair in limits)
                or any(not lo <= math.degrees(v) <= hi for q in points
                       for v, (lo, hi) in zip(q, limits))):
            raise BridgeRejected('Trajectory outside finite configured joint limits')
        driver = self.controller.adapter.trajectory_status('right')
        if (not isinstance(driver, dict) or not driver.get('driver_instance')
                or driver.get('status') not in ('idle', 'failed', 'reference_complete')):
            raise BridgeRejected('No idle trajectory driver')
        r = dict(phase=phase, plan=copy.deepcopy(plan), binding=copy.deepcopy(binding),
                 driver_instance=driver['driver_instance'], driver_epoch=driver['command_epoch'],
                 policy=asdict(self.policy), controller_sources=self.sources.copy(),
                 controller_config_hash=digest(self.controller.config))
        r['request_hash'] = digest(r)
        return r, values

    def start(self, phase, plan, binding):
        with self.lock:
            if self.status != 'idle':
                raise BridgeRejected('Segment already used or latched; never replay')
            now = self.controller.clock()
            state = self.controller.snapshot()
            self.request, self.frozen = self._request(phase, plan, binding, state, now)
            try:
                self._approve(state, now)
            except Exception as error:
                raise BridgeRejected(str(error)) from error
            self.token = self.controller.reserve_motion(state['command_epoch'])
            self.status = 'dispatching'
            self.start_time = self.last_poll = now
            try:
                self.handle = self.controller.start_trajectory(
                    'right', copy.deepcopy(self.request['plan']), self.token,
                    preflight=self._approve,
                    expected_driver={'driver_instance':self.request['driver_instance'],
                                     'command_epoch':self.request['driver_epoch']})
                after_ack = self.controller.clock()
                if (not finite(after_ack) or not now <= after_ack <= now+self.policy.max_poll_gap_s
                        or not self.controller.motion_valid(self.token)):
                    raise BridgeRejected('Acknowledgement outlived dispatch lease or ownership')
                self._approve(self.controller.snapshot(), after_ack)
                self.last_poll = after_ack
                self.status = 'running'
                return self.snapshot()
            except Exception as error:
                self.outcome_unknown = True
                self.cancel('Dispatch outcome unknown: '+str(error))
                raise BridgeRejected(self.reason) from error

    def cancel(self, reason='manual stop'):
        """Issue stop before acquiring admission/controller/confirmation locks."""
        self.cancel_event.set()
        send = False
        with self.stop_dispatch_lock:
            if self.status in self.TERMINAL:
                return self.snapshot()
            if self.token is not None and not self.stop_sent and not self.stop_inflight:
                self.stop_inflight = True
                self.stop_request_time = self.controller.clock()
                send = True
        # No bridge/controller lock may delay the independent service request.
        if send:
            try:
                self.controller.adapter.arm_stop('right')
                with self.stop_dispatch_lock:
                    self.stop_sent = True
                    self.stop_error = ''
            except Exception as error:
                with self.stop_dispatch_lock:
                    self.stop_error = str(error)[:300]
            finally:
                with self.stop_dispatch_lock:
                    self.stop_inflight = False
        # The event/service above never waits for handoff. Commit cancellation's
        # rest/history boundary under the same lock as ownership handoff. If
        # handoff already committed verified arrival, completion linearizes
        # first; cancellation_requested remains latched for the parent task.
        if not self.stop_lock.acquire(blocking=False):
            return self.snapshot()
        try:
            if self.status in self.TERMINAL:
                return self.snapshot()
            self.reason = str(reason)[:300]
            if self.token is None:
                self.status = 'cancelled'
                return self.snapshot()
            if self.status not in ('stopping','recovery_required') or send:
                self.cancel_generation += 1
                self.rest_after = self.controller.clock()
                self.rest.clear()
            self.status = 'recovery_required' if self.outcome_unknown else 'stopping'
        finally:
            self.stop_lock.release()
        # If admission currently holds controller.lock, its cancellation check
        # prevents publication when it returns. The bound driver epoch also
        # prevents an old start from restarting a driver that processed stop.
        if self.controller.lock.acquire(blocking=False):
            try:
                if self.token is self.controller.motion_owner and self.controller.motion_valid(self.token):
                    self.controller.interrupt_motion(self.token)
            finally:
                self.controller.lock.release()
        return self.snapshot()

    def _rested(self, state, now, *, target=None):
        with self.stop_lock:
            return self._observe_rest(state,now,target=target)

    def _observe_rest(self, state, now, *, target=None):
        return measured_rest(state, now, self.policy, self.rest_after, self.rest, target=target)

    def _release(self, *, completion=False):
        if self.outcome_unknown:
            return
        generation = self.cancel_generation
        def still_stopped(state, now):
            try:
                stamps, values = self._feedback(state, now)
                driver = self.controller.adapter.trajectory_status('right')
                return (bool(self.rest) and self._sources_valid() and generation==self.cancel_generation
                        and (not completion or not self.cancel_event.is_set())
                        and stamps == self.rest[-1][0] and values == self.rest[-1][1]
                        and isinstance(driver, dict)
                        and driver.get('driver_instance') == self.request['driver_instance']
                        and driver.get('status') in ('idle', 'failed', 'reference_complete'))
            except BridgeRejected:
                return False
        # Atomically bind handoff to the sample that proved rest. No sent-stop
        # or a previously observed stationary pose can substitute for this check.
        with self.stop_lock, self.controller.lock:
            if self.token is not self.controller.motion_owner:
                raise BridgeRejected('Controller ownership changed; recovery required')
            if not still_stopped(self.controller.snapshot(), self.controller.clock()):
                raise BridgeRejected('Feedback or driver changed before control handoff')
            if self.controller.motion_trajectory is not None:
                self.controller.clear_stopped_trajectory(self.token, verify_stopped=still_stopped)
            self.controller.release_motion(self.token)
            self.token = None

    def poll(self):
        with self.lock:
            if self.status in self.TERMINAL or self.status == 'idle':
                return self.snapshot()
            now = self.controller.clock()
            state = self.controller.snapshot()
            if self.status in ('running', 'awaiting_arrival'):
                try:
                    if (not finite(now) or now < self.last_poll
                            or now-self.last_poll > self.policy.max_poll_gap_s
                            or now-self.start_time >= self.policy.timeout_s):
                        raise BridgeRejected('Clock, task deadline or poll lease expired')
                    self.last_poll = now
                    self._approve(state, now)
                    if not self.controller.motion_valid(self.token):
                        raise BridgeRejected('Manual/parent interruption latched')
                    if self.status == 'running':
                        result = self.controller.poll_trajectory(self.token, preflight=self._approve)
                        if result['status'] == 'reference_complete':
                            self.status = 'awaiting_arrival'
                            self.rest_after = now
                            self.rest.clear()
                    if self.status == 'awaiting_arrival':
                        driver = self.controller.adapter.trajectory_status('right')
                        if (not self.handle or not driver or driver.get('status') != 'reference_complete'
                                or any(driver.get(k) != self.handle[k] for k in ('request_id', 'driver_instance'))
                                or driver.get('command_epoch') != self.handle['command_epoch']+1):
                            raise BridgeRejected('Terminal trajectory receipt changed')
                        if self._rested(state, now, target=self.request['plan']['joint_waypoints_rad'][-1]):
                            self._release(completion=True)
                            if self.token is None:
                                self.status = 'completed'
                    return self.snapshot()
                except Exception as error:
                    return self.cancel(str(error))
            # Cancellation never becomes success, even after verified rest.
            driver = self.controller.adapter.trajectory_status('right')
            same_driver_stopped = (isinstance(driver, dict) and self.request is not None
                                   and driver.get('driver_instance') == self.request['driver_instance']
                                   and driver.get('status') in ('idle', 'failed', 'reference_complete'))
            if self.stop_sent and same_driver_stopped and self._rested(state, now):
                if not self.outcome_unknown:
                    try:
                        self._release()
                        if self.token is None:
                            self.status = 'cancelled'
                    except Exception as error:
                        self.stop_error = str(error)[:300]
            return self.snapshot()


def wheels_stopped(values):
    """Allow +/-0.05 in native wheel-feedback units, including encoder zero residue."""
    return ControllerBridge._vector(values, 4) and all(abs(value) <= .05 for value in values)


def measured_feedback(state, now, policy):
    try:
        right, left, body = state['arms']['right'], state['arms']['left'], state['body']
        stamps = [right['received'], left['received'], body['received']]
        if (not finite(now) or state.get('chassis_active') is not False
                or any(row.get('fresh') is not True for row in (right, left, body))
                or not all(finite(v) and 0 <= now-v <= policy.max_age_s for v in stamps)
                or max(stamps)-min(stamps) > policy.max_age_s
                or not ControllerBridge._vector(right['joints'], 6) or not ControllerBridge._vector(left['joints'], 6)
                or not ControllerBridge._vector(body['wheels'], 4)
                or not wheels_stopped(body['wheels'])):
            raise BridgeRejected('Fresh paired arm/body feedback and stationary chassis required')
        vector = (right['joints'] + [right['gripper'], body['height']]
                  + left['joints'] + [body['waist'], body['head_yaw'], body['head_pitch']])
        if not ControllerBridge._vector(vector, 17):
            raise BridgeRejected('Invalid measured feedback')
        return tuple(stamps), vector
    except (KeyError, TypeError) as error:
        raise BridgeRejected('Missing measured feedback') from error


def measured_rest(state, now, policy, rest_after, samples, *, target=None):
    try:
        stamps, values = measured_feedback(state, now, policy)
        if min(stamps) <= rest_after:
            return False
        if samples and any(t <= old for t, old in zip(stamps, samples[-1][0])):
            return False
        if target is not None and any(abs(a-b) > policy.joint_tolerance_rad
                                      for a, b in zip(values[:6], target)):
            samples.clear()
            return False
        stamp = min(stamps)
        if samples:
            old_stamps, old_values = samples[-1]
            dt = [a-b for a,b in zip(stamps,old_stamps)]
            components = [0]*7+[2]+[1]*6+[2]*3
            if (min(dt) <= 0 or max(dt) > policy.max_age_s
                    or any(abs(a-b)/dt[index] > policy.rest_speed_rad_s
                           for a,b,index in zip(values,old_values,components))):
                samples.clear()
        samples.append((stamps, values))
        return len(samples) >= 3 and stamp-min(samples[0][0]) >= policy.settle_s-1e-9
    except BridgeRejected:
        samples.clear()
        return False
