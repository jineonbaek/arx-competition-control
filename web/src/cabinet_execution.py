"""One horizontal placement task using the existing controller and arrival monitors."""
import copy
import json
from pathlib import Path
import threading
import time
import uuid

import numpy as np

from agent.cabinet_controller_bridge import BridgePolicy, digest, measured_rest, wheels_stopped
from agent.motion_feedback import ArrivalMonitor
from agent.storage import RunStore, atomic_json

ROOT = Path(__file__).resolve().parents[2]
TERMINAL = ('idle', 'blocked', 'cancelled', 'completed', 'completed_hold_outside', 'recovery_required')


class CabinetExecutionTask:
    def __init__(self, controller, preview, *, directory=None, planner=None,
                 verifier=None, auto_worker=True):
        self.ctrl, self.preview = controller, preview
        self.directory = Path(directory or ROOT/'web/hardware/runtime/cabinet-execution')
        self.planner = planner or self._plan
        self.verifier = verifier or (lambda *args: preview.verify_placement(*args))
        self.auto_worker = auto_worker
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.thread = self.token = self.store = None
        self.sleep = time.sleep
        self.arrival_policy = BridgePolicy(.002, .02, .02, .02, .3, .2, .2, 70.)
        self.state = dict(status='idle', phase='idle', reason='', battery_placed=False,
                          execution_enabled=controller.config['execution_enabled'],
                          physical_provider_configured=False, execution_backend='existing_controller',
                          start_checks_available=True, outside_tolerance_mm=5., model='demo', logs=[])
        try:
            old = json.loads((self.directory/'receipt.json').read_text())
            if not isinstance(old, dict) or old.get('model', 'demo') not in ('demo', 'competition'):
                raise ValueError('Invalid saved placement receipt')
            self.state['model'] = old.get('model', 'demo')
            if old.get('status') not in TERMINAL or old.get('status') == 'recovery_required':
                self.state.update(status='recovery_required', reason='Previous task unfinished; inspect physical state, never replay')
        except FileNotFoundError:
            pass
        except (ValueError, TypeError, OSError):
            self.state.update(status='recovery_required', reason='Task receipt unreadable')

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def _save(self, **changes):
        with self.lock:
            if changes.get('phase') != self.state['phase'] and 'phase' in changes:
                self.state['logs'].append(dict(time=time.time(), phase=changes['phase']))
            self.state.update(changes)
            self.directory.mkdir(parents=True, exist_ok=True)
            atomic_json(self.directory/'receipt.json', self.state)

    def action(self, data):
        if data == {'op': 'cancel'}:
            self.cancel.set()
            if self.store is not None:
                (self.store.path/'cancel.flag').touch(mode=0o600)
            # Stop publication immediately; the worker owns final cleanup and handoff.
            token = self.token
            if token is not None and self.ctrl.motion_valid(token):
                self.ctrl.submit(dict(type='arm.stop', arm='right'), token)
            return self.snapshot()
        if (type(data) is not dict or set(data) != {'op', 'model'} or data['op'] not in ('start','align_held','test_held')
                or data['model'] not in ('demo', 'competition')):
            raise ValueError('Only current model start or cancel is accepted')
        return self._start(data['model'],keep_grip=data['op']!='start',align_only=data['op']=='align_held')

    def start_owned(self, token, model=None):
        return self._start(model or self.state['model'], token=token)

    def _start(self, model, *, token=None, keep_grip=False, align_only=False):
        with self.lock:
            if self.state['status'] not in ('idle', 'blocked', 'cancelled', 'completed','completed_hold_outside'):
                raise ValueError('Task active or recovery unresolved')
            if self.thread is not None and self.thread.is_alive():
                raise ValueError('Prior worker has not exited')
            state = self.ctrl.snapshot()
            if token is not None and not self.ctrl.motion_valid(token):
                raise ValueError('Parent motion ownership expired')
            self.owns_motion = token is None
            self.token = token or self.ctrl.reserve_motion(state['command_epoch'])
            self.cancel = threading.Event()
            self.store = None
            self.initial = copy.deepcopy(state)
            if hasattr(self, 'assets'):
                del self.assets
            self.source_hash = digest(self.ctrl.config)
            self._save(status='planning', phase='d435_scan', reason='', model=model,
                       run_id=uuid.uuid4().hex, battery_placed=False, released=False,
                       release_requested=False, motion_started=False, keep_grip=keep_grip, align_only=align_only,
                       shallow_insertion_mm=10. if keep_grip else None, control_retained=False, logs=[])
            if self.auto_worker:
                self.thread = threading.Thread(target=self._run, name='cabinet-placement', daemon=True)
                self.thread.start()
            return self.snapshot()

    def _check(self, state=None, now=None):
        if self.cancel.is_set() or not self.ctrl.motion_valid(self.token):
            raise ValueError('Placement cancelled or motion ownership interrupted')
        state = state or self.ctrl.snapshot()
        if digest(self.ctrl.config) != self.source_hash:
            raise ValueError('Controller configuration changed during placement')
        right, left, body = state['arms']['right'], state['arms']['left'], state['body']
        if (not all(row.get('fresh') for row in (right, left, body))
                or state.get('chassis_active') is not False or not wheels_stopped(body['wheels'])):
            raise ValueError('Fresh feedback and stationary chassis required')
        base = self.initial
        if (abs(body['height']-base['body']['height']) > .05
                or max(abs(a-b) for a,b in zip(left['joints'], base['arms']['left']['joints'])) > np.radians(.5)
                or any(abs(body[k]-base['body'][k]) > .02 for k in ('waist','head_yaw','head_pitch'))):
            raise ValueError('Lift, left arm or camera configuration changed')
        if not self.state['release_requested'] and abs(right['gripper']-base['arms']['right']['gripper']) > .05:
            raise ValueError('Holding gripper changed before release')
        return True

    def _check_assets(self):
        if hasattr(self, 'assets') and self.preview._source_hashes('d435') != self.assets:
            raise ValueError('Cabinet model, TCP or calibration changed')

    def _move(self, segment):
        self._save(phase=segment['phase'], status='running')
        self._check_assets()
        self._check()
        self._save(motion_started=True)
        self.ctrl.start_trajectory('right', segment['plan'], self.token, preflight=self._check)
        deadline = self.ctrl.clock()+self.arrival_policy.timeout_s
        while self.ctrl.clock() < deadline:
            self._check()
            result = self.ctrl.poll_trajectory(self.token, preflight=self._check)
            if result['status'] == 'reference_complete':
                break
            self.sleep(.02)
        else:
            raise ValueError('Placement trajectory timed out')
        # Use the original cabinet bridge's joint arrival and measured-rest checks.
        rest_after, rest = self.ctrl.clock(), []
        while self.ctrl.clock() < deadline:
            self._check()
            state = self.ctrl.snapshot()
            if measured_rest(state, self.ctrl.clock(), self.arrival_policy, rest_after, rest,
                             target=segment['plan']['joint_waypoints_rad'][-1]):
                if segment['phase'] == 'outside_align':
                    tool = self.ctrl.models['right'].wrist_transform(state['arms']['right']['joints'],
                               state['body']['height'])@self.bundle['tcp_matrix']
                    error = float(np.linalg.norm(tool[:3,3]*1000-np.asarray(segment['target_tcp_mm'])))
                    if not np.isfinite(error) or error > self.state['outside_tolerance_mm']+1e-9:
                        raise ValueError('Outside TCP arrival exceeds 5 mm; insertion not started')
                return
            self.sleep(.02)
        raise ValueError('Placement joint arrival timed out')

    def _release(self):
        if self.state.get('keep_grip'):raise ValueError('Keep-grip rehearsal cannot release battery')
        self._save(phase='release')
        self._check_assets()
        self._check()
        target = self.ctrl.config['arms']['right']['gripper_open_raw']
        # Once publication is attempted, an uncertain reply must never replay release.
        self._save(release_requested=True)
        result = self.ctrl.submit(dict(type='gripper', arm='right', position=target),
                                  self.token, preflight=self._check)
        if result.get('ok') is not True:
            raise ValueError('Gripper release receipt unknown')
        monitor = ArrivalMonitor([target], started_at=self.ctrl.clock(),
                                 **self.pick_policy['gripper_policy'])
        while True:
            self._check()
            arm = self.ctrl.snapshot()['arms']['right']
            receipt = monitor.update([arm['gripper']], arm['received'], self.ctrl.clock())
            if receipt['arrived']:
                self._save(released=True)
                return
            if receipt['stop_required']:
                raise ValueError('Gripper opening not confirmed: '+receipt['status'])
            self.sleep(.02)

    def _run(self):
        status, reason, retain_motion = 'completed', '', False
        try:
            from agent.execution_policy import load
            self.pick_policy = load(self.preview.root/'config/battery-execution.json',
                gripper_limits=self.ctrl.config['arms']['right']['gripper_limits_raw'])['pick']
            self.bundle = self.planner(self.state['model'])
            self._check()
            parts=self.bundle['segments']
            if self.state.get('align_only'):parts=[part for part in parts if part['phase']=='outside_align']
            if not parts:raise ValueError('No outside alignment segments')
            for segment in parts:
                self._move(segment)
            if self.state.get('keep_grip'):
                if not self.state.get('align_only'):
                    # Reverse the exact checked quintic curve/timing, never open the claw.
                    for part in reversed([part for part in parts if part['phase']=='insert']):
                        plan=part['plan'];end=plan['time_from_start_s'][-1]
                        reverse=dict(interpolation=plan['interpolation'],
                            joint_waypoints_rad=list(reversed(plan['joint_waypoints_rad'])),
                            time_from_start_s=[end-t for t in reversed(plan['time_from_start_s'])])
                        self._move(dict(phase='retreat',plan=reverse))
                self._check()
                self._save(phase='outside_hold',reason='Held battery rehearsal ended outside; gripper never released')
                status='completed_hold_outside'
                return
            self._release()
            # Replan from measured insertion arrival; withdraw along the same axis and Z.
            for segment in self._retreat():
                self._move(segment)
            self._save(phase='verify')
            self._check_assets()
            self.store = RunStore.create(self.directory/'runs')
            proof = self.verifier(self.store, self.bundle['observation'], self.token, self._check, self.sleep)
            if proof.get('placement_confirmed') is not True:
                raise ValueError('Battery placement has not been visually confirmed')
            self._save(battery_placed=True, verification=proof)
        except Exception as error:
            reason = str(error)[:400]
            status = ('cancelled' if self.cancel.is_set() else
                      'recovery_required' if self.state['motion_started'] else 'blocked')
            if self.state['motion_started']:
                if self.state.get('keep_grip'):
                    retain_motion=True;status='recovery_required'
                try:
                    self.ctrl.submit(dict(type='arm.stop', arm='right'), self.token)
                except Exception as stop_error:
                    reason += '; stop failed: '+str(stop_error)[:100]
        finally:
            if self.store is not None:
                self.store.close()
            if self.owns_motion and not retain_motion:
                try:
                    self.ctrl.release_motion(self.token)
                except Exception as error:
                    status, reason = 'recovery_required', reason+'; control handoff failed: '+str(error)[:100]
            if retain_motion:self._save(control_retained=True)
            self._save(status=status, phase='completed' if status=='completed' else self.state['phase'], reason=reason)

    def close(self):
        if self.thread is not None and self.thread.is_alive():
            self.action({'op':'cancel'})
            self.thread.join(2.)

    def _segments(self, phase, path):
        points, times = path['joint_waypoints_rad'], path['timing']['time_from_start_s']
        model = self.ctrl.models['right']
        begin = 0
        while begin < len(points)-1:
            end = begin+1
            if times[end]-times[begin] > 45:
                raise ValueError('Individual segment exceeds supervised duration')
            while end+1 < len(points) and times[end+1]-times[begin] <= 45:
                end += 1
            tool = model.wrist_transform(points[end], self.initial['body']['height'])@self.bundle['tcp_matrix']
            xyz = (tool[:3,3]*1000).tolist()
            if phase == 'outside_align' and end == len(points)-1:
                xyz = self.bundle['observation']['plan'].get('tcp_pre_base_mm', xyz)
            yield dict(phase=phase, target_tcp_mm=xyz, target_rotation=tool[:3,:3].tolist(),
                       plan=dict(interpolation='segment_quintic_scalar',
                                 time_from_start_s=[t-times[begin] for t in times[begin:end+1]],
                                 joint_waypoints_rad=points[begin:end+1]))
            begin = end

    def _path(self, q, target):
        from agent.grasp_path import plan_correction
        actual = self.ctrl.models['right'].wrist_transform(q, self.initial['body']['height'])@self.bundle['tcp_matrix']
        if np.allclose(actual, target, rtol=0., atol=1e-9):
            return dict(joint_waypoints_rad=[list(q),list(q)], timing={'time_from_start_s':[0.,.2]})
        return plan_correction(self.ctrl.models['right'], q, self.initial['body']['height'],
            self.bundle['tcp_matrix'], target, self.ctrl.config['arms']['right']['joint_limits_deg'],
            timing_limits=self.bundle['timing'])

    def _plan(self, model):
        result = self.preview.action({'camera':'d435','model':model}, include_arm_preview=False, token=self.token)
        if self.state.get('keep_grip') and result.get('window'):
            from agent.cabinet_preview_geometry import horizontal_plan,opening
            root=self.preview.root
            slot=opening(json.loads((root/'config/cabinet-profiles.json').read_text()),model)
            result['plan']=horizontal_plan(result['window'],slot,
                json.loads((root/'config/battery-envelope.json').read_text()),
                tip_point_object_mm=json.loads((root/'web/public/cad/battery-grip.json').read_text())['points_object_mm']['tip'],
                preclearance_mm=100.,max_insertion_depth_mm=10.)
            result['status']='localized'
        if result.get('status') != 'localized' or not result.get('plan'):
            raise ValueError('D435 path rejected: '+result.get('reason', result.get('status','unknown')))
        self.assets = result['source_hashes']
        self._check()
        p = result['plan']
        tcp = np.asarray(json.loads((self.preview.root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
        self.bundle = dict(tcp_matrix=tcp, observation=result,
            timing=dict(joint_speed_deg_s=[3.]*6, joint_acceleration_deg_s2=[6.]*6,
                        tcp_speed_mm_s=12.,tcp_acceleration_mm_s2=25.))
        target = np.eye(4)
        target[:3,:3] = p['R_base_from_tool']
        target[:3,3] = np.asarray(p['tcp_pre_base_mm'])/1000
        q = self.ctrl.snapshot()['arms']['right']['joints']
        outside = self._path(q, target)
        if self.state.get('keep_grip'):
            from agent.cabinet_loaded_sweep import check_outside_sweep
            battery=json.loads((self.preview.root/'config/battery-envelope.json').read_text())
            battery['contact_mm']=json.loads((self.preview.root/'web/public/cad/battery-grip.json').read_text())['points_object_mm']['tip']
            self.bundle['loaded_sweep']=check_outside_sweep(self.ctrl.models['right'],self.ctrl.models['left'],
                self.initial['arms']['left']['joints'],self.initial['body']['height'],outside['joint_waypoints_rad'],
                tcp,battery,result['window'],result.get('table_estimate',{}))
        if self.state.get('align_only'):
            self.bundle['segments']=list(self._segments('outside_align',outside));return self.bundle
        from agent.grasp_path import plan_insert
        insert = plan_insert(self.ctrl.models['right'], outside['joint_waypoints_rad'][-1],
            self.initial['body']['height'], p['tcp_pre_base_mm'], p['tcp_end_base_mm'],
            p['R_base_from_tool'], tcp, self.ctrl.config['arms']['right']['joint_limits_deg'],
            timing_limits=(dict(self.bundle['timing'],tcp_speed_mm_s=3.,tcp_acceleration_mm_s2=6.)
                           if self.state.get('keep_grip') else self.bundle['timing']))
        self.bundle['segments'] = list(self._segments('outside_align', outside))+list(self._segments('insert', insert))
        return self.bundle

    def _retreat(self):
        state = self.ctrl.snapshot()
        target = self.ctrl.models['right'].wrist_transform(state['arms']['right']['joints'],
                    self.initial['body']['height'])@self.bundle['tcp_matrix']
        p = self.bundle['observation']['plan']
        target[:3,3] -= np.asarray(p['insertion_axis_base'])*p['stroke_mm']/1000
        return self._segments('retreat', self._path(state['arms']['right']['joints'], target))
