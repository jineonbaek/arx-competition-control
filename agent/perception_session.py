"""One lazy model process per automatic task; file IPC never sends robot commands."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
from .contracts import AgentError
from .storage import atomic_json

ROOT=Path(__file__).resolve().parents[1]
FP=Path('/home/arx/arx-foundationpose/foundationpose')


class PerceptionSession:
    def __init__(self,run):
        self.directory=Path(run).resolve()/'perception'
        self.child=None

    def call(self,operation,arguments,cancelled,timeout):
        if cancelled():raise AgentError('cancelled','视觉任务已取消')
        if self.child is None:
            self.directory.mkdir(exist_ok=True)
            with (self.directory/'worker.log').open('a') as log:
                self.child=subprocess.Popen(['bash','-c','source "$1"; exec "$2" -m agent.perception_worker "$3"',
                    'perception-worker',str(FP/'env.sh'),str(FP/'.venv/bin/python'),str(self.directory)],
                    cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        request_id=uuid.uuid4().hex
        response=self.directory/(request_id+'.json')
        atomic_json(self.directory/'request.json',dict(id=request_id,operation=operation,arguments=arguments))
        started=time.monotonic()
        while not response.exists():
            if cancelled():
                self.close();raise AgentError('cancelled','视觉任务已取消')
            if self.child.poll() is not None:
                raise AgentError('vision_failed','视觉进程已退出；请查看 perception/worker.log')
            if time.monotonic()-started>timeout:
                self.close();raise AgentError('worker_timeout','模型处理超时')
            time.sleep(.05)
        reply=json.loads(response.read_text())
        if reply['id']!=request_id:raise ValueError('Perception receipt mismatch')
        if not reply['ok']:raise AgentError('vision_failed',reply['error'])
        return reply['data']

    def close(self):
        if self.child is None:return
        if self.child.poll() is None:
            os.killpg(self.child.pid,signal.SIGTERM)
            try:self.child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid,signal.SIGKILL);self.child.wait()
        self.child=None
