"""Fixed competition sequence; modules own motion details and recovery evidence."""
import copy
import json
import threading
import time
import uuid
from pathlib import Path

from control import Rejected
from agent.storage import atomic_json


STAGES = (
    ('navigate_pick', '导航到取物工作区并停稳'),
    ('pick', '抓取并抽出指定电池'),
    ('carry', '进入运输姿态'),
    ('navigate_place', '携物通过障碍区并到柜前停稳'),
    ('place', '红框 A 口插入、释放并稳定 3 秒'),
)
ACTIVE = ('running', 'recovering', 'stopping')
LEFT_HOLD_TIMEOUT_S = 60.


class Interrupted(Exception):
    pass


class CompetitionTask:
    stages = STAGES
    def __init__(self, controller, root, *, modules=None, recovery=None, navigation=None):
        self.controller, self.root = controller, Path(root)
        self.modules = dict(modules or {})
        self.recovery = recovery
        self.navigation = navigation
        self.lock = threading.RLock()
        self.active = None
        self.requests = {}

    def register(self, stage, module):
        """Process-only integration point; no browser can register an actuator."""
        with self.lock:
            if self.active and (self.active['status'] in ACTIVE or self.active['worker'].is_alive()):
                raise Rejected('比赛任务运行中，不能替换模块')
            if stage not in dict(self.stages):
                raise ValueError('Unknown competition stage')
            if not all(callable(getattr(module, k, None)) for k in ('start', 'inspect', 'stop', 'resume')):
                raise ValueError('Module requires start, inspect, stop and resume')
            self.modules[stage] = module

    def snapshot(self, client=None):
        with self.lock:
            s = self.active
            robot = self.controller.snapshot()
            manual_available = not robot['motion_reserved'] and not robot['chassis_active'] and not robot['competition_reserved']
            view = dict(status=s['status'] if s else 'idle',
                run_id=s['run_id'] if s else None,
                phase=('finish_forward' if s and (s['records'].get('finish_forward') or {}).get('status')=='running' else self.stages[s['index']][0] if s and s['index'] < len(self.stages) else 'idle'),
                reason=s['reason'] if s else '',
                command_epoch=robot['command_epoch'],
                control_owned=bool(s and s['competition_token'] is not None),
                manual_available=manual_available,
                can_start=manual_available and robot['execution_enabled'] and
                    (not s or (s['status'] not in ACTIVE and s['competition_token'] is None and not s['worker'].is_alive())),
                can_continue=bool(s and s['status'] in ('paused', 'needs_attention')
                    and manual_available and robot['execution_enabled']
                    and s['competition_token'] is None and not s['worker'].is_alive()),
                tasks=[], logs=copy.deepcopy(s['logs'][-50:]) if s else [],
                target='无人机指定左上角电池 → 电池柜红框 A 口')
            for key, label in self.stages:
                record = (s['records'].get(key) or {}) if s else {}
                view['tasks'].append(dict(id=key, label=label, connected=key in self.modules,
                    status=record.get('status', 'pending'), request_id=record.get('request_id'),
                    reason=record.get('reason', '')))
            if s and 'finish_forward' in s['records']:
                record=s['records']['finish_forward'];view['tasks'].append(dict(id='finish_forward',label='末点后向前0.225 SDK前进4秒并停车',connected=True,status=record.get('status','pending'),request_id=record['request_id'],reason=''))
            if self.navigation:view['navigation']=self.navigation.snapshot()
            return view

    def _save(self, s, event, detail=''):
        with self.lock:
            item = dict(time=time.time(), event=event, detail=str(detail)[:500])
            s['logs'].append(item)
            view = self.snapshot()
            atomic_json(s['directory'] / 'state.json', view)
            with (s['directory'] / 'events.jsonl').open('a', encoding='utf-8') as out:
                out.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + '\n')

    def action(self, client, data):
        if not isinstance(client, str) or not client or not isinstance(data, dict):
            raise ValueError('Valid web session and object required')
        action, request = data.get('action'), data.get('request_id')
        if not isinstance(request, str) or not 1 <= len(request) <= 128:
            raise ValueError('request_id required')
        expected = {'action', 'request_id'}
        if action in ('start', 'continue'):
            expected |= {'expected_epoch', 'environment_clear'}
        if set(data) != expected or action not in ('start', 'stop', 'continue'):
            raise ValueError('Unknown competition action or fields')
        with self.lock:
            old = self.requests.get((client, request))
            if old:
                if old[0] != data:
                    raise Rejected('同一请求编号不能用于不同操作')
                return copy.deepcopy(old[1])
            if action == 'stop':
                s = self.active
                if s is None:
                    raise Rejected('没有比赛任务')
                s['cancel'].set()
                if s['worker'].is_alive():
                    s['status'] = 'stopping'
                    threading.Thread(target=self._stop_module, args=(s,), daemon=True).start()
                elif s['competition_token'] is not None:
                    self._finish(s, 'paused', '操作员停止')
                self._save(s, 'stop_requested')
            else:
                if type(data['expected_epoch']) is not int or data['environment_clear'] is not True:
                    raise ValueError('沿用既有作业区确认后才能开始或继续')
                if self.active and (self.active['worker'].is_alive() or self.active['competition_token'] is not None):
                    raise Rejected('已有比赛任务或待清理控制权')
                if action == 'continue':
                    s = self.active
                    if not s or s['status'] not in ('paused', 'needs_attention'):
                        raise Rejected('没有可继续的中断任务')
                else:
                    run_id = 'competition-' + uuid.uuid4().hex
                    directory = self.root / run_id
                    directory.mkdir(parents=True, mode=0o700)
                    s = dict(run_id=run_id, directory=directory, index=0, records={}, logs=[],
                        current=None, halt_lock=threading.Lock(), token=None, competition_token=None,
                        worker=threading.Thread(), reason='', status='idle', lift_prepositioned=False)
                lease = self.controller.reserve_competition(data['expected_epoch'])
                s.update(token=None, competition_token=lease, cancel=threading.Event(), status='running', reason='', current=None)
                self.active = s
                worker = threading.Thread(target=self._run, args=(s,), daemon=True, name='competition-task')
                s['worker'] = worker
                try:
                    self._save(s, action)
                    pick=self.modules.get('pick')
                    if s['index']<=1 and callable(getattr(pick,'prewarm',None)):
                        pick.prewarm(s['cancel'])
                    worker.start()
                except Exception:
                    s['status'] = 'needs_attention'
                    if s.get('lift_prepositioned'):
                        pick.stop_preposition_lift(self.controller,lease)
                        s['lift_prepositioned']=False
                    self.controller.release_competition(lease)
                    s['competition_token'] = None
                    raise
            result = self.snapshot(client)
            self.requests[(client, request)] = (copy.deepcopy(data), copy.deepcopy(result))
            return result

    def _context(self, s, stage, record):
        return dict(t2_timed_out=((s['records'].get('place') or {}).get('result') or {}).get('outcome')=='timed_out',run_id=s['run_id'], stage=stage, request_id=record['request_id'],
            token=s['token'], cancelled=s['cancel'], controller=self.controller,
            environment_clear=True, competition_token=s['competition_token'])

    def _check(self, s, *, reading=False):
        if s['cancel'].is_set():
            raise Interrupted('操作员停止')
        if not self.controller.competition_valid(s['competition_token']):
            if not reading:raise Interrupted('整场任务已被停止或控制权失效')
        if self.navigation:self.navigation.check()
        if s['token'] is not None and not self.controller.motion_valid(s['token']):
            if reading and self.controller.motion_owner is s['token']:
                return
            raise Interrupted('任务控制权被人工停止或已失效')

    def _mode(self, s, navigation):
        if navigation:
            if s['token'] is not None:
                self.controller.release_motion(s['token']);s['token']=None
            self.controller.set_competition_navigation(s['competition_token'],True)
        else:
            self.controller.set_competition_navigation(s['competition_token'],False)
            if s['token'] is None:
                s['token']=self.controller.reserve_motion(self.controller.snapshot()['command_epoch'],
                    competition=s['competition_token'])

    def _prepare_arm_presets(self, s):
        self._mode(s, False)
        prestarted = s['token'].prestarted_lift_target
        if prestarted is not None:
            s['competition_token'].prestarted_lift_target = prestarted
            s['token'].prestarted_lift_target = None
        self.controller.submit({'type':'arm.fixed_hold2', 'arm':'left'}, s['token'])
        self.controller.submit({'type':'arm.avoidance_preset', 'arm':'right'}, s['token'])
        self._save(s, 'left_hold_requested', 'J1-J6 0/0/90/0/0/0 deg; gripper closed')
        self._save(s, 'right_avoidance_requested', 'J1-J6 0/0/65/-65/0/0 deg; gripper unchanged')
        deadline = time.monotonic() + LEFT_HOLD_TIMEOUT_S
        while time.monotonic() < deadline:
            self._check(s)
            snapshot = self.controller.snapshot()
            left = snapshot['left_fixed_hold2']
            right = snapshot['right_avoidance_preset']
            if left['arrived'] and right['arrived']:
                self._save(s, 'left_hold_ready', 'max joint error %.3f deg' % left['max_error_deg'])
                self._save(s, 'right_avoidance_ready', 'max joint error %.3f deg' % right['max_error_deg'])
                self.controller.release_motion(s['token'])
                s['token'] = None
                return
            s['cancel'].wait(.05)
        raise ValueError('左右臂预设点位 60 秒内未进入每轴 ±2°')

    def _wait(self, s, function):
        """Cancellation can discard a slow model reply; it never executes tools itself."""
        done, result = threading.Event(), []
        def call():
            try: result.append((True, function()))
            except Exception as error: result.append((False, error))
            finally: done.set()
        threading.Thread(target=call, daemon=True).start()
        while not done.wait(.1):
            self._check(s, reading=True)
        self._check(s, reading=True)
        ok, value = result[0]
        if not ok: raise value
        return value

    def _execute(self, s, stage, module, record):
        # Placement stores a NavigationSession in this attribute. Only the
        # explicit stage flag may hand control to navigation and drop the arm lease.
        self._mode(s,getattr(module,'navigation',False) is True)
        context = self._context(s, stage, record)
        with self.lock:
            self._check(s)
            s['current'] = (module, context)
            already_started = record.get('started', False)
            record['started'] = True
            record['status'] = 'running'
            self._save(s, 'module_inspect' if already_started else 'module_start', record['request_id'])
        if already_started:
            result = module.inspect(context)
            if result.get('status') == 'completed':
                return result
            if result.get('status') == 'stopped' and result.get('can_resume') is True:
                self._check(s)
                result = module.resume(context)
            elif result.get('status') != 'running':
                raise ValueError(result.get('reason') or '原任务状态无法核实，不能重复启动')
        else:
            if self.navigation and self.navigation.active:self.navigation.before_work(context,stage)
            if stage=='navigate_place':
                self.controller.submit_competition_lift({'type':'lift','target':0.},s['competition_token'])
                s['lift_prepositioned']=True
                self._save(s,'placement_lift_requested','0 rad; continue navigation without waiting')
            result = module.start(context)
            if stage=='pick':s['lift_prepositioned']=False
        waiting_since=time.monotonic();last_wait=None
        while True:
            if not isinstance(result, dict):
                raise ValueError('模块返回无效结果')
            if result.get('timeout_phase'):
                record['reason']=result.get('reason','')
                if record.get('timeout_phase')!=result['timeout_phase']:
                    record['timeout_phase']=result['timeout_phase'];self._save(s,'t2_timeout_progress',record['reason'])
            status = result.get('status')
            if status in ('completed', 'failed', 'stopped', 'unknown'):
                return result
            self._check(s)
            if status not in ('running', 'accepted'):
                raise ValueError('模块返回未知状态')
            marker=(result.get('status'),result.get('reason'))
            if marker!=last_wait:last_wait=marker;waiting_since=time.monotonic()
            elif time.monotonic()-waiting_since>=15.:
                from agent.recovery_watch import report_fault
                report_fault(stage,result.get('reason') or '模块持续运行，尚无完成回执',run=s['run_id'])
                waiting_since=time.monotonic()
            s['cancel'].wait(.1)
            self._check(s)
            result = module.inspect(context)

    def _recover(self, s, stage, module, record, failure):
        self._check(s, reading=True)
        context = self._context(s, stage, record)
        inspection = module.inspect(context)
        recoveries = getattr(module, 'recoveries', {})
        if not isinstance(recoveries, dict): raise ValueError('恢复任务注册表无效')
        if self.recovery is None:
            raise ValueError('异常恢复模型未接入：' + str(failure))
        s['status'] = 'recovering'
        evidence = dict(stage=stage, request_id=record['request_id'], failure=str(failure)[:500],
            module_state=inspection, robot=self._robot_evidence(),
            completed=[k for k, r in s['records'].items() if r.get('status') == 'completed'])
        self._save(s, 'recovery_analysis', str(failure))
        decision = self._wait(s, lambda:self.recovery.decide(evidence, list(recoveries)))
        name = decision.get('name') if isinstance(decision, dict) else None
        self._save(s, 'recovery_decision', json.dumps(decision, ensure_ascii=False)[:500])
        if name == 'stop' or name not in recoveries:
            raise ValueError((decision or {}).get('reason') or '没有明确且已接入的恢复能力')
        # An unresolved earlier recovery must be inspected, never started again.
        recovery_record = record.setdefault('recovery', dict(name=name,
            request_id=record['request_id'] + ':recovery:' + uuid.uuid4().hex))
        if recovery_record['name'] != name:
            raise ValueError('先核实原恢复任务，不能替换未决动作')
        result = self._execute(s, stage, recoveries[name], recovery_record)
        if result.get('status') != 'completed':
            raise ValueError(result.get('reason') or '恢复任务未完成')
        result = module.inspect(context)
        if result.get('status') == 'completed':
            return result
        if result.get('status') == 'stopped' and result.get('can_resume') is True:
            s['status'] = 'running'
            return self._execute(s, stage, module, record)
        raise ValueError(result.get('reason') or '恢复后仍无法核实原任务完成状态')

    def _robot_evidence(self):
        snapshot = self.controller.snapshot()
        return {k:snapshot.get(k) for k in ('arms', 'body', 'motion_reserved', 'motion_interrupted', 'chassis_active')}

    def _prepare_run(self,s):
        prepare=getattr(self.navigation,'prepare',None)
        preflight=getattr(self.navigation,'validate_start_route',None)
        if s['index']==0:
            if callable(prepare):prepare(self._context(s,'navigate_pick',dict(request_id=s['run_id']+':navigate_pick')))
            elif callable(preflight):preflight()
        self._prepare_arm_presets(s)
        pick = self.modules.get('pick')
        if (s['index']==0 and callable(getattr(pick,'preposition_lift',None))):
            self.controller.set_competition_navigation(s['competition_token'],True)
            pick.preposition_lift(self.controller,s['competition_token'])
            s['lift_prepositioned']=True

    def _run(self, s):
        status, reason = 'completed', ''
        try:
            missing = [label for key, label in self.stages if key not in self.modules]
            if missing:
                raise ValueError(missing[0] + '：能力未接入')
            self._prepare_run(s)
            self._save(s, 'current_state_checked', json.dumps(self._robot_evidence(), ensure_ascii=False)[:500])
            while s['index'] < len(self.stages):
                self._check(s)
                stage = self.stages[s['index']][0]
                module = self.modules.get(stage)
                if module is None:
                    raise ValueError(dict(self.stages)[stage] + '：能力未接入')
                record = s['records'].setdefault(stage, dict(request_id=s['run_id'] + ':' + stage))
                try:
                    result = self._execute(s, stage, module, record)
                    if result.get('status') != 'completed':
                        raise ValueError(result.get('reason') or '任务没有完成回执')
                except Interrupted:
                    raise
                except Exception as error:
                    from agent.recovery_watch import report_fault
                    report_fault(stage,str(error),run=s['run_id'])
                    result = self._recover(s, stage, module, record, error)
                    if result.get('status') != 'completed':
                        raise ValueError(result.get('reason') or '恢复后任务仍未完成')
                with self.lock:
                    # Preserve a completed receipt even if Stop raced its arrival.
                    record.update(status='completed', result=copy.deepcopy(result))
                    if result.get('outcome')=='timed_out':record['reason']=result.get('reason','T2超时已跳过')
                    s['index'] += 1
                    s['current'] = None
                    s['status'] = 'running'
                    self._save(s, 'module_completed', stage)
            self._check(s)
            if self.navigation and self.navigation.active:
                self._mode(s,True)
                self.navigation.complete_route(self._context(s,'place',s['records']['place']))
            if self.navigation and callable(getattr(self.navigation,'finish_forward',None)):
                self._check(s);self._mode(s,False)
                record=s['records'].setdefault('finish_forward',dict(request_id=s['run_id']+':finish_forward'))
                record['status']='running';self._save(s,'final_forward_started','末点完成：0.225 SDK车头正前方4秒')
                self.navigation.finish_forward(self._context(s,'finish_forward',record))
                record['status']='completed';self._save(s,'final_forward_completed','前进4秒并结束输出')
        except Interrupted as error:
            status, reason = 'paused', str(error)
        except Exception as error:
            status, reason = 'needs_attention', str(error)[:500]
        finally:
            self._finish(s, status, reason)

    def _stop_module(self, s):
        with s['halt_lock']:
            current = s['current']
            if current is None: return True
            if s.get('stop_ack') is current: return True
            module, context = current
            try:
                result = module.stop(context)
                if not isinstance(result, dict) or result.get('stopped') is not True:
                    raise ValueError('当前模块停止未确认')
                s['stop_ack'] = current
                return True
            except Exception as error:
                s['reason'] = '停止失败：' + str(error)[:300]
                return False

    def _finish(self, s, status, reason):
        stopped = status == 'completed' or self._stop_module(s)
        pick=self.modules.get('pick')
        if s.get('lift_prepositioned'):
            try:
                pick.stop_preposition_lift(self.controller,s['competition_token'])
                s['lift_prepositioned']=False
            except Exception as error:
                stopped=False
                reason='预升降停止失败：'+str(error)[:300]
        try:
            if callable(getattr(pick,'discard_prewarm',None)):pick.discard_prewarm()
        except Exception as error:self.controller.log('视觉预热清理失败：'+str(error)[:200],'warning')
        with self.lock:
            if s['cancel'].is_set(): status, reason = 'paused', '操作员停止'
            s.update(status=status, reason=reason or s['reason'])
            if s['index'] < len(self.stages):
                record = s['records'].get(self.stages[s['index']][0])
                if record and record.get('status') != 'completed':
                    record.update(status='stopped' if status=='paused' else 'failed', reason=s['reason'])
            if s['token'] is not None:
                try:
                    if not stopped: raise Rejected(s['reason'])
                    self.controller.release_motion(s['token'])
                    s['token'] = None
                except Exception as error:
                    stopped=False
                    s.update(status='needs_attention', reason='控制权待清理：' + str(error)[:300])
            if stopped and self.navigation and self.navigation.active:
                try:self.navigation.stop(self._context(s,'stop',dict(request_id=s['run_id']+':stop')))
                except Exception as error:
                    stopped=False;s.update(status='needs_attention',reason='导航控制权待清理：'+str(error)[:300])
            if stopped and s['competition_token'] is not None:
                try:
                    self.controller.release_competition(s['competition_token']);s['competition_token']=None
                    self.controller.log('整场任务已退出，控制权已返还 9983')
                except Exception as error:s.update(status='needs_attention',reason='控制权待清理：'+str(error)[:300])
            if s['status']=='needs_attention':
                from agent.recovery_watch import report_fault
                report_fault('整场/控制权清理',s['reason'],run=s['run_id'])
            self._save(s, 'competition_' + s['status'], s['reason'])

    def close(self):
        with self.lock:
            s = self.active
            if not s: return
            s['cancel'].set()
        self._stop_module(s)
        if s['worker'].is_alive(): s['worker'].join(3)
