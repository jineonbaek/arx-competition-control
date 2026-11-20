"""Local ARX control console transport and low-speed planar command."""
import json
import math
import threading
import time
import uuid
from dataclasses import dataclass
from contextlib import contextmanager
from http.cookiejar import CookieJar
from urllib.request import build_opener, HTTPCookieProcessor, ProxyHandler, Request


@dataclass(frozen=True)
class OutputTask:
    source_id: str
    task_id: str
    generation: int


class WebChassis:
    sdk_units = False
    native_sdk = False
    origin = 'http://127.0.0.1:9983'
    command_kind = None

    def __init__(self, submit=None, *, source_id=None, wall=time.time):
        self.submit=submit;self.wall=wall;self.lock=threading.RLock();self.context=threading.local()
        self.source_id=source_id or ('work-' if submit else 'navigation-')+uuid.uuid4().hex
        self.generation=0;self.sequences={};self.ended=set();self.current=None
        if submit is None:
            self.client=build_opener(ProxyHandler({}),HTTPCookieProcessor(CookieJar()))
            with self.client.open(self.origin+'/',timeout=.2):pass
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
                    issued_at=self.wall(),ttl_s=.3)

    @contextmanager
    def task_scope(self,handle):
        previous=getattr(self.context,'handle',None)
        self.context.handle=handle
        try:yield handle
        finally:self.context.handle=previous

    def _send(self,payload):
        if self.submit is not None:result=self.submit(payload)
        else:
            request=Request(self.origin+'/api/command',json.dumps(payload,allow_nan=False).encode(),
                            {'Origin':self.origin,'Content-Type':'application/json'})
            with self.client.open(request,timeout=.18) as reply:result=json.load(reply)
        if not result.get('ok'):raise RuntimeError('控制台拒绝了底盘指令')
        return result

    def send(self,velocity,handle=None):
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
            return self._send(payload)

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
