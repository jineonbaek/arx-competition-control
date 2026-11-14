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


class Interrupted(Exception):
    pass


class CompetitionTask:
    def __init__(self, controller, root, *, modules=None, recovery=None):
        self.controller, self.root = controller, Path(root)
        self.modules = dict(modules or {})
        self.recovery = recovery
        self.lock = threading.RLock()
        self.active = None
        self.requests = {}

    def register(self, stage, module):
        """Process-only integration point; no browser can register an actuator."""
        with self.lock:
            if self.active and (self.active['status'] in ACTIVE or self.active['worker'].is_alive()):
                raise Rejected('比赛任务运行中，不能替换模块')
            if stage not in dict(STAGES):
                raise ValueError('Unknown competition stage')
            if not all(callable(getattr(module, k, None)) for k in ('start', 'inspect', 'stop', 'resume')):
                raise ValueError('Module requires start, inspect, stop and resume')
            self.modules[stage] = module

    def snapshot(self, client=None):
        with self.lock:
            s = self.active
            robot = self.controller.snapshot()
            manual_available = not robot['motion_reserved'] and not robot['chassis_active']
            view = dict(status=s['status'] if s else 'idle',
                run_id=s['run_id'] if s else None,
                phase=STAGES[s['index']][0] if s and s['index'] < len(STAGES) else 'idle',
                reason=s['reason'] if s else '',
                command_epoch=robot['command_epoch'],
                control_owned=bool(s and s['token'] is not None),
                manual_available=manual_available,
                can_start=manual_available and robot['execution_enabled'] and
                    (not s or (s['status'] not in ACTIVE and s['token'] is None and not s['worker'].is_alive())),
                can_continue=bool(s and s['status'] in ('paused', 'needs_attention')
                    and manual_available and robot['execution_enabled']
                    and s['token'] is None and not s['worker'].is_alive()),
                tasks=[], logs=copy.deepcopy(s['logs'][-50:]) if s else [],
                target='无人机指定左上角电池 → 电池柜红框 A 口')
            for key, label in STAGES:
                record = (s['records'].get(key) or {}) if s else {}
                view['tasks'].append(dict(id=key, label=label, connected=key in self.modules,
                    status=record.get('status', 'pending'), request_id=record.get('request_id'),
                    reason=record.get('reason', '')))
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
                elif s['token'] is not None:
                    self._finish(s, 'paused', '操作员停止')
                self._save(s, 'stop_requested')
            else:
                if type(data['expected_epoch']) is not int or data['environment_clear'] is not True:
                    raise ValueError('沿用既有作业区确认后才能开始或继续')
                if self.active and (self.active['worker'].is_alive() or self.active['token'] is not None):
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
                        current=None, halt_lock=threading.Lock(), token=None,
                        worker=threading.Thread(), reason='', status='idle')
                token = self.controller.reserve_motion(data['expected_epoch'])
                s.update(token=token, cancel=threading.Event(), status='running', reason='', current=None)
                self.active = s
                worker = threading.Thread(target=self._run, args=(s,), daemon=True, name='competition-task')
                s['worker'] = worker
                try:
                    self._save(s, action)
                    worker.start()
                except Exception:
                    s['status'] = 'needs_attention'
                    self.controller.release_motion(token)
                    s['token'] = None
                    raise
            result = self.snapshot(client)
            self.requests[(client, request)] = (copy.deepcopy(data), copy.deepcopy(result))
            return result

    def _context(self, s, stage, record):
        return dict(run_id=s['run_id'], stage=stage, request_id=record['request_id'],
            token=s['token'], cancelled=s['cancel'], controller=self.controller,
            environment_clear=True)

    def _check(self, s, *, reading=False):
        if s['cancel'].is_set():
            raise Interrupted('操作员停止')
        if not self.controller.motion_valid(s['token']):
            if reading and self.controller.motion_owner is s['token']:
                return
            raise Interrupted('任务控制权被人工停止或已失效')

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
            result = module.start(context)
        while True:
            if not isinstance(result, dict):
                raise ValueError('模块返回无效结果')
            status = result.get('status')
            if status in ('completed', 'failed', 'stopped', 'unknown'):
                return result
            self._check(s)
            if status not in ('running', 'accepted'):
                raise ValueError('模块返回未知状态')
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

    def _run(self, s):
        status, reason = 'completed', ''
        try:
            self._save(s, 'current_state_checked', json.dumps(self._robot_evidence(), ensure_ascii=False)[:500])
            while s['index'] < len(STAGES):
                self._check(s)
                stage = STAGES[s['index']][0]
                module = self.modules.get(stage)
                if module is None:
                    raise ValueError(dict(STAGES)[stage] + '：能力未接入')
                record = s['records'].setdefault(stage, dict(request_id=s['run_id'] + ':' + stage))
                try:
                    result = self._execute(s, stage, module, record)
                    if result.get('status') != 'completed':
                        raise ValueError(result.get('reason') or '任务没有完成回执')
                except Interrupted:
                    raise
                except Exception as error:
                    result = self._recover(s, stage, module, record, error)
                    if result.get('status') != 'completed':
                        raise ValueError(result.get('reason') or '恢复后任务仍未完成')
                with self.lock:
                    # Preserve a completed receipt even if Stop raced its arrival.
                    record.update(status='completed', result=copy.deepcopy(result))
                    s['index'] += 1
                    s['current'] = None
                    s['status'] = 'running'
                    self._save(s, 'module_completed', stage)
            self._check(s)
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
        with self.lock:
            if s['cancel'].is_set(): status, reason = 'paused', '操作员停止'
            s.update(status=status, reason=reason or s['reason'])
            if s['index'] < len(STAGES):
                record = s['records'].get(STAGES[s['index']][0])
                if record and record.get('status') != 'completed':
                    record.update(status='stopped' if status=='paused' else 'failed', reason=s['reason'])
            if s['token'] is not None:
                try:
                    if not stopped: raise Rejected(s['reason'])
                    self.controller.release_motion(s['token'])
                    s['token'] = None
                    self.controller.log('整场任务已退出，控制权已返还 9983')
                except Exception as error:
                    s.update(status='needs_attention', reason='控制权待清理：' + str(error)[:300])
            self._save(s, 'competition_' + s['status'], s['reason'])

    def close(self):
        with self.lock:
            s = self.active
            if not s: return
            s['cancel'].set()
        self._stop_module(s)
        if s['worker'].is_alive(): s['worker'].join(3)
