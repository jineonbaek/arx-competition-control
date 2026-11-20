"""Task-scoped chassis client; ROS domain191 transport is independent of 9983."""
import json
import math
import threading
import time
import uuid
from dataclasses import dataclass
from contextlib import contextmanager
from http.cookiejar import CookieJar
from urllib.request import build_opener,HTTPCookieProcessor,ProxyHandler


class OutputUnavailable(TimeoutError):
    """Only retry after scoped cleanup and a new stopped/source proof."""

_OUTPUT_FAILURES=frozenset((
    '底盘指令已过期或早于维护结束','底盘指令在输出前已过期','底盘任务已结束或指令序号已过期',
    'ROS chassis scheduler unavailable','ROS chassis management timeout',
    'ROS chassis command acknowledgement timeout','ROS chassis management failed',
    'driver not ready or scheduler not bound','no bound driver',
    'scheduler instance changed','scheduler restarted; release controls and begin a new task',
    'client session unknown; re-register without replaying old commands',
))

def output_unavailable(error):
    return isinstance(error,OutputUnavailable) or str(error) in _OUTPUT_FAILURES


class _ConsoleStateReader:
    """Legacy status readers only; chassis commands stay on ROS domain191."""
    def __init__(self,origin):
        self.origin=origin;self.ready=False
        self.opener=build_opener(ProxyHandler({}),HTTPCookieProcessor(CookieJar()))
    def open(self,url,timeout=.3):
        if not isinstance(url,str) or url!=self.origin+'/api/state':
            raise ValueError('Console compatibility reader accepts GET /api/state only')
        if not self.ready:
            with self.opener.open(self.origin+'/',timeout=timeout):pass
            self.ready=True
        return self.opener.open(url,timeout=timeout)


@dataclass(frozen=True)
class OutputTask:
    source_id: str
    task_id: str
    generation: int


class WebChassis:
    sdk_units = False
    native_sdk = False
    command_kind = None
    origin = 'http://127.0.0.1:9983'

    @property
    def client(self):
        with self.lock:
            if not hasattr(self,'_console_reader'):self._console_reader=_ConsoleStateReader(self.origin)
            return self._console_reader

    @client.setter
    def client(self,reader):
        self._console_reader=reader

    def __init__(self, submit=None, *, source_id=None, wall=time.time):
        self.submit=submit;self.wall=wall;self.lock=threading.RLock();self.context=threading.local()
        self.source_id=source_id or ('work-' if submit else 'navigation-')+uuid.uuid4().hex
        self.generation=0;self.sequences={};self.ended=set();self.current=None
        self.ros_session=None
        self.begin_task()

    def begin_task(self, task_id='output'):
        with self.lock:
            if self.current is not None and self.current not in self.ended:
                self.end_output(self.current)
            self.generation+=1
            self.current=OutputTask(self.source_id,task_id,self.generation)
            self.sequences[self.current]=0
            return self.current

    def _payload(self,kind,handle):
        if handle not in self.sequences:raise RuntimeError('unknown chassis task')
        self.sequences[handle]+=1
        return dict(type=kind,source_id=handle.source_id,task_id=handle.task_id,
                    generation=handle.generation,seq=self.sequences[handle],
                    issued_at=self.wall(),ttl_s=(1. if kind=='chassis.nav_sdk' or kind=='chassis.end_output' and (self.command_kind=='chassis.nav_sdk' or self.command_kind is None and self.sdk_units and self.native_sdk) else .3))

    @contextmanager
    def task_scope(self,handle):
        previous=getattr(self.context,'handle',None)
        self.context.handle=handle
        try:yield handle
        finally:self.context.handle=previous

    def _send(self,payload):
        if self.submit is not None:
            result=self.submit(payload)
        else:
            import sys
            from pathlib import Path
            root=Path(__file__).resolve().parents[2]
            sys.path.insert(0,str(root/'chassis_runtime'))
            from arx_chassis.client import Session
            kind=payload['type']
            configured_kind=self.command_kind or ('chassis.nav_sdk' if self.sdk_units and self.native_sdk else 'chassis')
            profile={'chassis.nav_sdk':'navigation','chassis.gamepad_sdk':'gamepad'}.get(configured_kind,'manual')
            if self.ros_session is None:self.ros_session=Session(profile)
            p=dict(payload)
            if kind=='chassis':
                scale=json.loads((root/'web/config/control.json').read_text())['body'].get('chassis_input_scale',2.5)/2.5
                for k in ('x','y','omega'):p[k]*=scale
            result=self.ros_session.send(p)
        if not result.get('ok'):raise RuntimeError('ROS scheduler rejected chassis command')
        return result

    def send(self,velocity,handle=None,*,ttl_s=None):
        if ttl_s is not None and (not math.isfinite(ttl_s) or not 0<ttl_s<=.3):
            raise ValueError('invalid short chassis command lifetime')
        with self.lock:
            handle=(getattr(self.context,'handle',None) or self.current) if handle is None else handle
            if handle in self.ended:raise RuntimeError('chassis task already ended')
            x,y,yaw=map(float,velocity)
            if not all(math.isfinite(v) for v in (x,y,yaw)):raise ValueError('nonfinite chassis command')
            if self.sdk_units and not self.native_sdk:
                scale=min(2.5,1.999999/max(math.hypot(x,y),1e-9))
                x,y,yaw=x*scale,y*scale,yaw*2.5
            kind=self.command_kind or ('chassis.nav_sdk' if self.sdk_units and self.native_sdk else 'chassis')
            payload=self._payload(kind,handle);payload.update(x=x,y=y,omega=yaw)
            if ttl_s is not None:payload['ttl_s']=float(ttl_s)
            try:return self._send(payload)
            except Exception as exc:
                if output_unavailable(exc):raise OutputUnavailable(str(exc)) from exc
                raise

    def renew_task(self,handle):
        with self.lock:
            scoped=getattr(self.context,'handle',None)
            if self.current!=handle or scoped not in (None,handle):
                raise RuntimeError('chassis output task changed during recovery')
            if handle not in self.ended:raise RuntimeError('old chassis output end is unconfirmed')
            renewed=self.begin_task(handle.task_id)
            if scoped==handle:self.context.handle=renewed
            return renewed

    def end_output(self,handle=None):
        with self.lock:
            handle=(getattr(self.context,'handle',None) or self.current) if handle is None else handle
            if handle in self.ended:return
            # Keep the same identity on retries; never upgrade failure to global stop.
            result=self._send(self._payload('chassis.end_output',handle))
            self.ended.add(handle)
            return result


def follow_velocity(forward, left, yaw_error):
    gain = min(.8, .08 / max(math.hypot(forward, left), 1e-6))
    return forward * gain, left * gain, max(-.18, min(.18, yaw_error))
