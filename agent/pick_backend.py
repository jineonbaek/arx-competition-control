"""Single prepared pick exposed through the existing Agent backend contract.

Only deterministic code can supply the step and bind its exact plan/target payload.
No device setup, HTTP endpoint or default production registration lives here.
"""
import copy
import json
import math
import os
from pathlib import Path
from threading import Event, Lock, Thread
import time

from .storage import atomic_json


class PickExecutionBackend:
    def __init__(self,step,*,request_id,arguments,journal,cancelled,timeout_s):
        if (not isinstance(request_id,str) or not request_id or len(request_id)>256
                or not isinstance(arguments,dict) or not callable(cancelled)
                or type(timeout_s) not in (int,float) or not math.isfinite(timeout_s)
                or not 0<timeout_s<=3600):
            raise ValueError('Bound request, cancellation and finite execution deadline required')
        plan=arguments.get('plan',{})
        if (not isinstance(plan,dict) or plan.get('arm')!='right'
                or not arguments.get('plan_id') or plan.get('plan_id')!=arguments['plan_id']):
            raise ValueError('Prepared right-arm plan identity required')
        self.step=step
        self.request_id=request_id
        self.payload=json.dumps(arguments,sort_keys=True,allow_nan=False)
        self.journal=Path(journal)
        self.cancelled=cancelled
        self.timeout_s=float(timeout_s)
        self.cancel_event=Event()
        self.done=Event()
        self.lock=Lock()
        self.thread=None
        self.current=None

    def available(self,operation):return operation=='execute_pick'

    def _receipt(self,status,**data):
        return dict(status=status,execution_id=self.request_id,data=dict(grasp_confirmed=False,**data))

    def _save(self,receipt):
        with self.lock:
            atomic_json(self.journal,dict(payload=self.payload,receipt=receipt))
            self.current=copy.deepcopy(receipt)

    def invoke(self,operation,arguments,request_id):
        try:matches=json.dumps(arguments,sort_keys=True,allow_nan=False)==self.payload
        except (ValueError,TypeError):matches=False
        if operation!='execute_pick' or request_id!=self.request_id or not matches:
            return dict(status='rejected',execution_id=request_id,data={'reason':'bound_plan_mismatch'})
        with self.lock:
            if self.thread is not None:return copy.deepcopy(self.current)
            # Persist unknown before effects; a new process must query, never replay this journal.
            receipt=self._receipt('unknown',pick_stage='starting')
            try:
                with self.journal.open('x',encoding='utf-8') as handle:
                    json.dump(dict(payload=self.payload,receipt=receipt),handle,allow_nan=False)
                    handle.flush();os.fsync(handle.fileno())
                fd=os.open(self.journal.parent,os.O_RDONLY|os.O_DIRECTORY)
                try:os.fsync(fd)
                finally:os.close(fd)
            except FileExistsError:
                return self._stored()
            self.current=receipt
            self.thread=Thread(target=self._run,name='prepared-pick',daemon=True)
            try:self.thread.start()
            except Exception:
                self.done.set()
                raise
            return copy.deepcopy(receipt)

    def _stored(self):
        try:
            saved=json.loads(self.journal.read_text())
            receipt=saved['receipt']
            if (saved['payload']!=self.payload or receipt['execution_id']!=self.request_id
                    or receipt['status'] not in ('completed','rejected')):
                return self._receipt('unknown',pick_stage='recovery_required')
            return receipt
        except (OSError,ValueError,KeyError,TypeError):
            return self._receipt('unknown',pick_stage='receipt_unavailable')

    def status(self,operation,request_id):
        if operation!='execute_pick' or request_id!=self.request_id:
            return dict(status='unknown',execution_id=request_id,data={'reason':'unknown_request'})
        with self.lock:
            return copy.deepcopy(self.current) if self.current is not None else self._stored()

    def _run(self):
        try:
            if self.cancel_event.is_set() or self.cancelled():
                self._save(self._receipt('rejected',pick_stage='cancelled_before_start'))
                return
            deadline=time.monotonic()+self.timeout_s
            receipt=self.step.start()
            last=None
            while True:
                stage=receipt['status']
                if receipt.get('stop_required'):
                    self._save(self._receipt('unknown',pick_stage=stage,
                        stop_request=receipt.get('stop_request'),recovery_required=True))
                    return
                if stage=='awaiting_hold_verification':
                    self._save(self._receipt('completed',pick_stage=stage,
                                            pose_reached=True,gripper_settled=True))
                    return
                if stage not in ('awaiting_trajectory','awaiting_driver_status','trajectory_running',
                                 'inserting','ready_to_close','closing'):
                    raise ValueError('Unexpected pick execution stage')
                if stage!=last:
                    self._save(self._receipt('running',pick_stage=stage));last=stage
                if (self.cancel_event.wait(.02) or self.cancelled() or time.monotonic()>=deadline):
                    receipt=self.step.cancel()
                else:receipt=self.step.poll()
        except Exception as error:
            try:stop=self.step.cancel().get('stop_request')
            except Exception:stop='failed'
            receipt=self._receipt('unknown',pick_stage='execution_unknown',
                stop_request=stop,recovery_required=True,reason=str(error)[:180])
            # Even a journal failure must not leave an earlier running/success receipt in memory.
            with self.lock:self.current=receipt
            try:self._save(receipt)
            except OSError:pass
        finally:self.done.set()

    def wait(self,timeout):
        return self.done.wait(timeout)

    def cancel_and_wait(self,timeout):
        self.cancel_event.set()
        return self.thread is None or self.wait(timeout)
