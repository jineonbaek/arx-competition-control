"""Bind durable Agent requests to the existing local Controller steps."""
import copy
import hashlib
import json
import math
from threading import Lock
import time
from .battery_execution import prepare_step
from .pick_backend import CoarseExecutionBackend, ApproachExecutionBackend, PickExecutionBackend

BACKENDS={c.operation:c for c in (CoarseExecutionBackend,ApproachExecutionBackend,PickExecutionBackend)}

class BatteryControllerBackend:
    def __init__(self,controller,token,store,*,policies,pick_setup,preflight,cancelled,timeout_s):
        if store.closed or not callable(preflight) or not callable(cancelled):
            raise ValueError('Open run and local preflight/cancellation required')
        if set(timeout_s)!=set(BACKENDS) or any(type(t) not in (int,float) or not math.isfinite(t)
                                               or not 0<t<=3600 for t in timeout_s.values()):
            raise ValueError('Finite deadlines required for all execution stages')
        self.controller=controller;self.token=token;self.store=store
        self.policies=copy.deepcopy(policies);self.pick_setup=copy.deepcopy(pick_setup)
        self.preflight=preflight;self.cancelled=cancelled;self.timeout_s=dict(timeout_s)
        self.directory=store.path/'executions';self.directory.mkdir(exist_ok=True)
        self.entries={};self.lock=Lock();self.cancelled_here=False

    def available(self,operation):
        return (operation in BACKENDS and not self.store.closed and not self.cancelled_here
                and not self.store.cancelled and not self.cancelled()
                and self.controller.motion_valid(self.token)
                and (operation=='execute_coarse_alignment' or self.pick_setup is not None))

    def _path(self,request):
        if not isinstance(request,str) or len(request)>256 or not request.startswith(self.store.state['run_id']+':'):
            raise ValueError('Request belongs to another run')
        return self.directory/(hashlib.sha256(request.encode()).hexdigest()+'.json')

    @staticmethod
    def _reply(status,request,reason):
        return dict(status=status,execution_id=request,data=dict(reason=reason,grasp_confirmed=False))

    def _stored(self,path,operation,request,payload=None):
        try:
            saved=json.loads(path.read_text());receipt=saved['receipt']
            if saved['operation']!=operation or receipt['execution_id']!=request:
                raise ValueError('Journal identity mismatch')
            if payload is not None and saved['payload']!=payload:
                return self._reply('rejected',request,'bound_plan_mismatch')
            if receipt['status'] in ('completed','rejected'):return receipt
        except (OSError,ValueError,KeyError,TypeError):pass
        return self._reply('unknown',request,'recovery_required')

    def invoke(self,operation,arguments,request_id):
        with self.lock:
            try:
                path=self._path(request_id)
                payload=json.dumps(arguments,sort_keys=True,allow_nan=False)
                if operation not in BACKENDS:raise ValueError('Unknown execution stage')
                if request_id in self.entries:
                    return self.entries[request_id].invoke(operation,arguments,request_id)
                if path.exists():return self._stored(path,operation,request_id,payload)
                if not self.available(operation):raise ValueError('Execution capability unavailable')
                pending=self.store.state.get('pending') or {}
                if (pending.get('request_id')!=request_id or pending.get('operation')!=operation
                        or pending.get('arguments')!=arguments):raise ValueError('Pending request mismatch')
                if arguments.get('plan')!=self.store.state['mission'].get('plan'):
                    raise ValueError('Mission plan mismatch')
                # An unverified earlier stop cannot be made safe by choosing a new request ID.
                for journal in self.directory.glob('*.json'):
                    saved=json.loads(journal.read_text())
                    if saved['receipt']['status'] not in ('completed','rejected'):
                        raise ValueError('Earlier execution requires recovery')
                step=prepare_step(operation,arguments['plan'],self.controller,self.token,
                    policies=self.policies,pick_setup=self.pick_setup,preflight=self.preflight)
                entry=BACKENDS[operation](step,request_id=request_id,arguments=arguments,journal=path,
                    cancelled=lambda:self.store.closed or self.store.cancelled or self.cancelled(),
                    timeout_s=self.timeout_s[operation])
                self.entries[request_id]=entry
                return entry.invoke(operation,arguments,request_id)
            except (OSError,ValueError,KeyError,TypeError) as error:
                # A journal may already exist if startup failed after durable intent was written.
                if 'path' in locals() and path.exists():
                    return self._reply('unknown',request_id,'recovery_required: '+str(error)[:160])
                return self._reply('rejected',request_id,str(error)[:180])

    def status(self,operation,request_id):
        with self.lock:
            if request_id in self.entries:return self.entries[request_id].status(operation,request_id)
            try:return self._stored(self._path(request_id),operation,request_id)
            except ValueError:return self._reply('unknown',request_id,'unknown_request')

    def cancel_and_wait(self,timeout):
        with self.lock:
            self.cancelled_here=True
            entries=list(self.entries.values())
        deadline=time.monotonic()+max(0,timeout)
        results=[entry.cancel_and_wait(max(0,deadline-time.monotonic())) for entry in entries]
        return all(results)
