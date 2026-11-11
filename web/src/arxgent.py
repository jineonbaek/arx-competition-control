"""Bounded browser conversations; the web Agent never receives a motion tool."""
import copy
import hashlib
from pathlib import Path
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agent.config import Config, load_private_env
from agent.contracts import AgentError, strict_json
from agent.runtime import AgentRuntime
from agent.storage import RunStore
from agent.tools import ToolRegistry

READ_ONLY_TOOLS = {'get_system_status', 'observe_scene', 'read_observation'}


class ChatError(Exception):
    def __init__(self, status, message):
        self.status, self.message = status, message
        super().__init__(message)


class ArxgentChat:
    registry_class = ToolRegistry
    allowed_tools = READ_ONLY_TOOLS
    request_limit = 20
    def __init__(self, config, root, *, runtime_factory=AgentRuntime, unavailable_reason=None):
        self.config, self.root, self.runtime_factory = config, Path(root), runtime_factory
        if config.motion_enabled or config.services:
            raise ValueError('Browser chat requires motion disabled and no domain backends')
        self.lock, self.sessions, self.worker_slot = threading.RLock(), {}, threading.BoundedSemaphore(1)
        self.closed, self.unavailable_reason = False, unavailable_reason

    @classmethod
    def from_repository(cls, root, port):
        root = Path(root)
        error = None
        try:
            private = root / 'agent/.secrets/qwen.env'
            if private.exists():
                load_private_env(private)
        except (OSError, ValueError, AgentError):
            error = 'arxgent 私有配置无法读取，请检查服务器配置与权限。'
        return cls(Config(console_url=f'http://127.0.0.1:{port}'), root / 'agent/runs', unavailable_reason=error)

    def _available(self):
        if self.closed:
            return False, 'arxgent 正在关闭，请稍后刷新。'
        if self.unavailable_reason:
            return False, self.unavailable_reason
        try:
            self.config.profile('cloud')
        except AgentError:
            return False, 'arxgent 尚未配置云端连接，请在服务器配置 API 密钥与地址。'
        return True, None

    @staticmethod
    def _new_session():
        return {'conversation_id': str(uuid.uuid4()), 'messages': [], 'job': None,
                'run_dir': None, 'requests': {}, 'activity': [], 'last_seen': time.monotonic()}

    def _session(self, client):
        now = time.monotonic()
        for identity, session in list(self.sessions.items()):
            busy = session['job'] and session['job']['status'] in ('running', 'cancelling')
            if identity != client and not busy and now - session['last_seen'] > 3600:
                del self.sessions[identity]
        if client not in self.sessions:
            if len(self.sessions) >= 32:
                unused = [(identity, session) for identity, session in self.sessions.items() if not session['messages'] and not session['job']]
                if not unused:
                    raise ChatError(503, '近期对话已达上限，请稍后再试。')
                del self.sessions[min(unused, key=lambda item: item[1]['last_seen'])[0]]
            self.sessions[client] = self._new_session()
        self.sessions[client]['last_seen'] = now
        return self.sessions[client]

    def _snapshot(self, session):
        available, reason = self._available()
        job = session['job']
        return {'ok': True, 'name': 'arxgent', 'model': self.config.profiles['cloud']['model'],
                'available': available, 'unavailable_reason': reason,
                'conversation_id': session['conversation_id'], 'messages': copy.deepcopy(session['messages']),
                'busy': bool(job and job['status'] in ('running', 'cancelling')),
                'job': copy.deepcopy(job), 'activity': copy.deepcopy(session['activity']),
                'read_only': True}

    def snapshot(self, client):
        with self.lock:
            return self._snapshot(self._session(client))

    def handle(self, client, data):
        if not isinstance(data, dict):
            raise ChatError(400, '请求需要 JSON 对象。')
        action = data.get('action')
        fields = {'send': {'action', 'conversation_id', 'request_id', 'message'},
                  'cancel': {'action', 'conversation_id', 'request_id'}, 'new': {'action', 'conversation_id'}}
        if not isinstance(action, str) or action not in fields or set(data) != fields[action]:
            raise ChatError(400, '对话请求字段无效。')
        with self.lock:
            session = self._session(client)
            if data['conversation_id'] != session['conversation_id']:
                raise ChatError(409, '对话已更新，请刷新对话后重试。')
            busy = bool(session['job'] and session['job']['status'] in ('running', 'cancelling'))
            if action == 'cancel':
                if not busy or data['request_id'] != session['job']['id']:
                    raise ChatError(409, '这条请求已经结束。')
                session['job']['status'] = 'cancelling'
                if session['run_dir']:
                    (Path(session['run_dir']) / 'cancel.flag').touch(mode=0o600)
                return self._snapshot(session)
            if action == 'new':
                if busy:
                    raise ChatError(409, '请先取消并等待当前回复结束。')
                self.sessions[client] = self._new_session()
                return self._snapshot(self.sessions[client])
            text, request_id = data['message'], data['request_id']
            if not isinstance(text, str) or not text.strip() or len(text) > 2000:
                raise ChatError(400, '请输入 1–2000 个字符。')
            try:
                if str(uuid.UUID(request_id)) != request_id:
                    raise ValueError()
            except (ValueError, TypeError, AttributeError):
                raise ChatError(400, '请求标识无效。') from None
            fingerprint = hashlib.sha256(text.encode()).hexdigest()
            if request_id in session['requests']:
                if session['requests'][request_id] != fingerprint:
                    raise ChatError(409, '同一请求标识不能发送不同内容。')
                return self._snapshot(session)
            available, reason = self._available()
            if not available:
                raise ChatError(503, reason)
            if busy:
                raise ChatError(409, 'arxgent 正在处理上一条消息。')
            if len(session['requests']) >= self.request_limit:
                raise ChatError(409, f'本对话已达 {self.request_limit} 轮，请开始新对话。')
            if not self.worker_slot.acquire(blocking=False):
                raise ChatError(503, 'arxgent 正在处理另一条对话，请稍后再试。')
            session['requests'][request_id] = fingerprint
            session['messages'].append({'role': 'user', 'content': text})
            session['activity'] = []
            session['job'] = {'id': request_id, 'status': 'running', 'phase': 'thinking', 'error': None, 'tools': []}
            worker = threading.Thread(target=self._work, args=(session, text), daemon=True, name='arxgent-chat')
            try:
                worker.start()
            except Exception:
                self.worker_slot.release()
                session['job'].update(status='error', error='无法启动对话，请开始新对话后重试。')
            return self._snapshot(session)

    def _progress(self, session, event):
        with self.lock:
            job = session['job']
            if job['status'] != 'cancelling':
                job['phase'] = event['event']
            name = event.get('tool')
            if event['event'] == 'tool' and name in self.allowed_tools:
                if name not in job['tools']:
                    job['tools'].append(name)
                session['activity'].append({'tool': name, 'status': 'running'})
                session['activity'] = session['activity'][-8:]
            if event['event'] == 'tool_result' and session['activity']:
                session['activity'][-1]['status'] = 'done' if event.get('ok') else 'error'

    def _work(self, session, text):
        started = time.monotonic()
        store = None
        answer, error, status = '', None, 'error'
        try:
            store = RunStore.open(session['run_dir']) if session['run_dir'] else RunStore.create(self.root)
            with self.lock:
                session['run_dir'] = str(store.path)
                if session['job']['status'] == 'cancelling' or self.closed:
                    (store.path / 'cancel.flag').touch(mode=0o600)
            tools = self.registry_class(self.config, store, allowed_tools=self.allowed_tools)
            runtime = self.runtime_factory(self.config, store, tools, progress=lambda event: self._progress(session, event),
                                           system_context='你的名字是 arxgent。你正在现有机器人网页控制台与用户连续对话。用简洁中文回答，可分点，但避免Markdown标题、表格或加粗。用户要求查看现场时，调用工具取得新图片；不要把旧观测当作实时。你只有只读工具，不能执行机器人运动。')
            result = runtime.continue_turn(text) if store.state['messages'] else runtime.run(text)
            if result['reason'] == 'model_finished':
                status, answer = 'done', result['answer']
            elif result['reason'] == 'cancelled':
                status = 'cancelled'
            else:
                error = {'model_http_error': '云端请求失败，请检查额度、密钥与服务状态后开始新对话。',
                         'model_unavailable': '云端连接暂不可用，请检查网络后开始新对话。',
                         'model_protocol_error': '云端回复不完整，请开始新对话后重试。'}.get(result['reason'], '本轮对话未完成，请开始新对话后重试。')
        except AgentError:
            error = '上轮对话未完整结束，请开始新对话后重试。'
        except Exception:
            error = 'arxgent 暂时无法完成回复，请开始新对话后重试。'
        finally:
            if store is not None:
                store.close()
            with self.lock:
                if session['job']['status'] == 'cancelling':
                    status, answer, error = 'cancelled', '', None
                if status == 'done':
                    session['messages'].append({'role': 'assistant', 'content': answer,
                                                'elapsed_s': round(time.monotonic() - started, 3),
                                                'tools': list(session['job']['tools'])})
                session['job'].update(status=status, phase=None, error=error)
            self.worker_slot.release()

    def close(self):
        with self.lock:
            self.closed = True
            for session in self.sessions.values():
                if session['job'] and session['job']['status'] in ('running', 'cancelling'):
                    session['job']['status'] = 'cancelling'
                    if session['run_dir']:
                        (Path(session['run_dir']) / 'cancel.flag').touch(mode=0o600)
