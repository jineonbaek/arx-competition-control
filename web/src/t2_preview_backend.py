"""Opt-in recorded T2 preview. Never constructs or calls a hardware worker."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from t2_preview import T2PreviewModule


def load_settings(path):
    path = Path(path)
    if not path.exists():
        return None
    settings = json.loads(path.read_text())
    if not isinstance(settings, dict):
        raise ValueError('T2 预览配置必须为对象')
    if settings.get('mode') == 'disabled':
        return None
    if settings.get('mode') != 'recorded_preview':
        raise ValueError('T2 模式无效；不回退到真实放置')
    for key in ('candidate_dir', 'workspace', 'frame', 'pose', 'control_config'):
        value = settings.get(key)
        if not isinstance(value, str) or not Path(value).is_absolute() or not Path(value).exists():
            raise ValueError('T2 历史预览路径无效：'+key)
    candidate = Path(settings['candidate_dir']) / 'recorded_t2_planner.py'
    if not candidate.is_file():
        raise ValueError('T2 历史预览规划器缺失')
    supports = settings.get('support_dirs')
    if not isinstance(supports, list) or not supports or any(
        not isinstance(p, str) or not Path(p).is_absolute() or not Path(p).is_dir() for p in supports
    ):
        raise ValueError('T2 历史预览依赖路径无效')
    timeout = settings.get('timeout_s', 30.)
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or not .1 <= timeout <= 60.:
        raise ValueError('T2 预览超时须在 0.1～60 秒之间')
    settings['timeout_s'] = timeout
    return settings


class _Cancelled:
    def __init__(self, *events):
        self.events = events

    def is_set(self):
        return any(event.is_set() for event in self.events)


class RecordedPreviewPlanner:
    def __init__(self, settings, report_root):
        self.settings = copy.deepcopy(settings)
        self.report_root = Path(report_root)

    def result_path(self, context):
        identity = json.dumps([context['run_id'], context['request_id']], ensure_ascii=False).encode()
        return self.report_root / (hashlib.sha256(identity).hexdigest()+'.json')

    def __call__(self, context):
        receipt = dict(competition_run_id=context['run_id'], request_id=context['request_id'],
            input_source='recorded', preview_only=True, executable=False, battery_placed=False,
            started_at=time.time(), status='failed',
            inputs={key:self.settings[key] for key in ('frame', 'pose', 'workspace')})
        process = None
        try:
            env = os.environ.copy()
            env.pop('PYTHONPATH', None)
            env['PYTHONDONTWRITEBYTECODE'] = '1'
            process = subprocess.Popen(
                [sys.executable, '-B', str(Path(__file__).with_name('t2_preview_runner.py'))],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, env=env, cwd=self.settings['workspace'], close_fds=True)
            deadline = time.monotonic()+self.settings['timeout_s']
            payload = json.dumps(self.settings)
            while True:
                if context['cancelled'].is_set():
                    raise ValueError('T2 历史预览已取消')
                if time.monotonic() >= deadline:
                    raise TimeoutError('T2 历史预览超时')
                try:
                    stdout, stderr = process.communicate(input=payload, timeout=.05)
                    break
                except subprocess.TimeoutExpired:
                    payload = None
            if process.returncode:
                raise ValueError('T2 历史预览进程失败：'+stderr[-1500:])
            preview = json.loads(stdout)
            if not isinstance(preview, dict):
                raise ValueError('T2 历史预览输出不是对象')
            receipt.update(preview=preview, reason='仅历史数据预览，未真实放置', status='preview_finished')
            return preview
        except Exception as error:
            receipt['reason'] = str(error)
            raise
        finally:
            if process is not None:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=.5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                process.communicate()
            receipt['finished_at'] = time.time()
            self.report_root.mkdir(parents=True, exist_ok=True)
            target = self.result_path(context)
            pending = target.with_suffix('.tmp')
            pending.write_text(json.dumps(receipt, ensure_ascii=False, allow_nan=False, indent=2))
            pending.replace(target)


class OnlineT2Preview(T2PreviewModule):
    preview_only = True

    def __init__(self, planner):
        super().__init__(planner, None)
        self.latest_context = None

    def _run(self, context, record):
        # The candidate's stop event must also terminate the isolated planner.
        super()._run(context | {'cancelled':_Cancelled(context['cancelled'], record['stop'])}, record)

    def start(self, context):
        result = super().start(context)
        with self.lock:
            self.latest_context = dict(context)
        return result

    def inspect(self, context):
        result = super().inspect(context)
        if result['status'] != 'unknown':
            result.update(preview_only=True, input_source='recorded', battery_placed=False,
                          result_path=str(self.planner.result_path(context)))
        return result

    def snapshot(self):
        with self.lock:
            result = self.inspect(self.latest_context) if self.latest_context else dict(status='idle')
        return dict(result, enabled=True, mode='recorded_preview', preview_only=True,
                    input_source='recorded', battery_placed=False,
                    frame=self.planner.settings['frame'], pose=self.planner.settings['pose'],
                    notice='T2 仅历史数据预览，不执行放置，不发送完成回执')

    def close(self):
        with self.lock:
            records = list(self.runs.values())
            for record in records:
                record['stop'].set()
        for record in records:
            record['worker'].join(2.)


def make_t2_placement(task, config_path, report_root):
    settings = load_settings(config_path)
    if settings is None:
        from competition_modules import PlacementModule
        return PlacementModule(task)
    return OnlineT2Preview(RecordedPreviewPlanner(settings, report_root))
