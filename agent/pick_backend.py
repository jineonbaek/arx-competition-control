"""Prepared motion stages exposed through the existing Agent backend contract.

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


class _PreparedExecutionBackend:
    """Shared durable execution loop; subclasses define the accepted stage contract."""
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

    def available(self,operation):return operation==self.operation

    def _receipt(self,status,*,stage,**data):
        return dict(status=status,execution_id=self.request_id,
                    data=dict(grasp_confirmed=False,**{self.stage_key:stage},**data))

    @staticmethod
    def _stop_data(receipt):
        return dict(stop_request=receipt.get('stop_request'),
                    stop_requests=receipt.get('stop_requests',{}),stopped_verified=False)

    def _save(self,receipt):
        with self.lock:
            atomic_json(self.journal,dict(operation=self.operation,payload=self.payload,receipt=receipt))
            self.current=copy.deepcopy(receipt)

    def invoke(self,operation,arguments,request_id):
        try:matches=json.dumps(arguments,sort_keys=True,allow_nan=False)==self.payload
        except (ValueError,TypeError):matches=False
        if operation!=self.operation or request_id!=self.request_id or not matches:
            return dict(status='rejected',execution_id=request_id,data={'reason':'bound_plan_mismatch'})
        with self.lock:
            if self.thread is not None:return copy.deepcopy(self.current)
            # Persist unknown before effects; a new process must query, never replay this journal.
            receipt=self._receipt('unknown',stage='starting')
            try:
                with self.journal.open('x',encoding='utf-8') as handle:
                    json.dump(dict(operation=self.operation,payload=self.payload,receipt=receipt),handle,allow_nan=False)
                    handle.flush();os.fsync(handle.fileno())
                fd=os.open(self.journal.parent,os.O_RDONLY|os.O_DIRECTORY)
                try:os.fsync(fd)
                finally:os.close(fd)
            except FileExistsError:
                return self._stored()
            self.current=receipt
            self.thread=Thread(target=self._run,name=self.operation,daemon=True)
            try:self.thread.start()
            except Exception:
                self.done.set()
                raise
            return copy.deepcopy(receipt)

    def _stored(self):
        try:
            saved=json.loads(self.journal.read_text())
            receipt=saved['receipt']
            if (saved.get('operation')!=self.operation or saved['payload']!=self.payload or receipt['execution_id']!=self.request_id
                    or receipt['status'] not in ('completed','rejected')):
                return self._receipt('unknown',stage='recovery_required')
            return receipt
        except (OSError,ValueError,KeyError,TypeError):
            return self._receipt('unknown',stage='receipt_unavailable')

    def status(self,operation,request_id):
        if operation!=self.operation or request_id!=self.request_id:
            return dict(status='unknown',execution_id=request_id,data={'reason':'unknown_request'})
        with self.lock:
            return copy.deepcopy(self.current) if self.current is not None else self._stored()

    def _run(self):
        try:
            if self.cancel_event.is_set() or self.cancelled():
                self._save(self._receipt('rejected',stage='cancelled_before_start'))
                return
            deadline=time.monotonic()+self.timeout_s
            receipt=self.step.start()
            last=None
            while True:
                stage=receipt['status']
                if receipt.get('stop_required'):
                    self._save(self._receipt('unknown',stage=stage,
                        **self._stop_data(receipt),recovery_required=True))
                    return
                if stage==self.terminal_stage:
                    self._save(self._receipt('completed',stage=stage,**self._completed_data(receipt)))
                    return
                if stage not in self.running_stages:
                    raise ValueError('Unexpected execution stage')
                if stage!=last:
                    self._save(self._receipt('running',stage=stage));last=stage
                cancelled=self.cancel_event.wait(.02) or self.cancelled()
                expired=time.monotonic()>=deadline
                if cancelled:
                    receipt=self.step.cancel()
                elif expired:
                    receipt=self.step.timeout() if hasattr(self.step,'timeout') else self.step.cancel()
                else:receipt=self.step.poll()
        except Exception as error:
            try:stop=self._stop_data(self.step.cancel())
            except Exception:stop=dict(stop_request='failed',stopped_verified=False)
            receipt=self._receipt('unknown',stage='execution_unknown',
                **stop,recovery_required=True,reason=str(error)[:180])
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


class PickExecutionBackend(_PreparedExecutionBackend):
    operation='execute_pick'
    stage_key='pick_stage'
    terminal_stage='awaiting_hold_verification'
    running_stages=('awaiting_trajectory','awaiting_driver_status','trajectory_running',
                    'inserting','ready_to_close','closing')

    @staticmethod
    def _completed_data(receipt):
        return dict(pose_reached=True,gripper_settled=True)


class CoarseExecutionBackend(_PreparedExecutionBackend):
    operation='execute_coarse_alignment'
    stage_key='coarse_stage'
    terminal_stage='ready_to_observe'
    running_stages=('lifting','observing_motion','settling_observer')

    @staticmethod
    def _completed_data(receipt):
        height=receipt.get('achieved_lift_rad')
        if (receipt.get('new_observation_required') is not True
                or type(height) not in (int,float) or not math.isfinite(height) or not 0<=height<=18):
            raise ValueError('Settled lift feedback and a new-observation barrier required')
        return dict(observer_ready=True,new_observation_required=True,achieved_lift_rad=height)


class ApproachExecutionBackend(_PreparedExecutionBackend):
    operation='execute_battery_approach'
    stage_key='approach_stage'
    terminal_stage='ready_to_reobserve'
    running_stages=('opening_gripper','awaiting_trajectory','awaiting_driver_status','trajectory_running','inserting')

    @staticmethod
    def _completed_data(receipt):
        if receipt.get('new_observation_required') is not True or receipt.get('close_permitted') is not False:
            raise ValueError('Measured arrival and mandatory reobservation required')
        return dict(pose_reached=True,new_observation_required=True)
