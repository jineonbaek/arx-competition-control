"""One-run visual docking coordinator with replaceable trusted internal backends.

Backend owns hardware reservation, checked full trajectories and fresh feedback.
This module never constructs a motor/ROS client and has no release operation.
"""
import copy
import json
import math
import time
from pathlib import Path
from .storage import atomic_json


class DockingRun:
    def __init__(self,backend,*,clock=time.time,journal=None,max_corrections=3):
        if type(max_corrections) is not int or not 1<=max_corrections<=3:raise ValueError('Correction budget is 1..3')
        self.backend=backend;self.clock=clock;self.journal=Path(journal) if journal else None
        self.max_corrections=max_corrections;self.driver=None;self.last_frame=None;self.consumed=set()
        self.seen_frames=set();self.last_capture=-math.inf;self.motion_finished_at=-math.inf
        self.state=dict(status='idle',phase='idle',released=False,motion_started=False,events=[])
        if self.journal and self.journal.exists():
            # Any prior invocation is evidence, never permission to replay a run.
            self.state.update(status='recovery_required',reason='Existing task receipt requires explicit physical recovery')

    def _save(self,**changes):
        self.state.update(changes)
        if self.journal:
            self.journal.parent.mkdir(parents=True,exist_ok=True);atomic_json(self.journal,self.state)

    def _check_cancel(self):
        if self.backend.cancelled():raise InterruptedError('Cancelled; no new motion')

    def _observe(self):
        self._check_cancel();obs=self.backend.observe();self._check_cancel()
        if (not isinstance(obs,dict) or not isinstance(obs.get('id'),str) or not obs['id']
                or obs['id'] in self.seen_frames or type(obs.get('captured_at')) not in (int,float)
                or not math.isfinite(obs['captured_at']) or not 0<=self.clock()-obs['captured_at']<=.8
                or obs['captured_at']<=max(self.last_capture,self.motion_finished_at)
                or obs.get('grip_stable') is not True or not isinstance(obs.get('driver_id'),str) or not obs['driver_id']):
            raise ValueError('Fresh independent vision and stable held-object feedback required')
        if self.driver is None:self.driver=obs['driver_id']
        if obs['driver_id']!=self.driver:raise ValueError('Driver changed; do not replay motion')
        self.last_frame=obs['id'];self.seen_frames.add(obs['id']);self.last_capture=obs['captured_at'];return obs

    def _execute(self,phase,obs,*,reverse_of=None):
        self._check_cancel();plan=self.backend.plan(phase,obs,reverse_of=reverse_of)
        if (not isinstance(plan,dict) or not isinstance(plan.get('id'),str) or not plan['id']
                or plan.get('phase')!=phase or plan.get('observation_id')!=obs['id']
                or plan.get('driver_id')!=self.driver or plan.get('gripper_action')!='retain'
                or plan.get('checked_sweep') is not True or plan.get('measured_feedback') is not True
                or plan.get('strategy') not in ('arm','base','lift')
                or type(plan.get('max_duration_s')) not in (int,float)
                or not math.isfinite(plan['max_duration_s']) or not 0<plan['max_duration_s']<=(300 if phase=='align' else 60)):
            raise ValueError('Checked bounded plan bound to current vision/driver required')
        if phase in ('insert','withdraw') and (plan.get('fixed_z') is not True or plan['strategy']=='lift'):
            raise ValueError('Insertion/withdrawal must preserve Z; no lift movement')
        if phase=='withdraw' and plan.get('reverse_of')!=reverse_of:raise ValueError('Withdrawal must bind the confirmed insertion')
        if plan['id'] in self.consumed:raise ValueError('Plan already consumed; no dispatch replay')
        if not 0<=self.clock()-obs['captured_at']<=.8:raise ValueError('Planning outlived its observation; refresh and revalidate before dispatch')
        self.consumed.add(plan['id'])
        self._save(phase=phase,status='running',motion_started=True,inflight=plan['id'])
        self._check_cancel()
        receipt=self.backend.execute(copy.deepcopy(plan),self._check_cancel)
        self.motion_finished_at=self.clock()
        self._check_cancel()
        if (not isinstance(receipt,dict) or receipt.get('plan_id')!=plan['id']
                or receipt.get('driver_id')!=self.driver or receipt.get('measured_complete') is not True
                or receipt.get('gripper_released') is not False
                or (phase in ('insert','withdraw') and receipt.get('fixed_z') is not True)
                or receipt.get('engagement')!=('inside' if phase=='insert' else 'outside')):
            raise ValueError('Physical outcome uncertain; stop, never blindly retry or reverse')
        self.state['events'].append(dict(phase=phase,strategy=plan['strategy'],plan_id=plan['id'],observation_id=obs['id']))
        self._save(inflight=None);return plan

    def run(self):
        if self.state['status']!='idle':return copy.deepcopy(self.state)
        try:
            self._save(status='observing',phase='observe');obs=self._observe();previous=None
            for attempt in range(self.max_corrections+1):
                if obs.get('outside') is not True:raise ValueError('Outside position must be known before alignment')
                fit=obs.get('fit',{})
                if obs.get('at_entry') is True and fit.get('qualified_fit') is True:break
                error=fit.get('available_clearance_mm')
                if previous is not None and (type(error) not in (int,float) or not math.isfinite(error) or error<=previous+.1):
                    raise ValueError('Outside correction made no measurable fit progress')
                if attempt>=self.max_corrections:raise ValueError('Bounded outside correction budget exhausted')
                previous=error if type(error) in (int,float) and math.isfinite(error) else None
                self._execute('align',obs);obs=self._observe()
            insertion=self._execute('insert',obs)
            obs=self._observe()
            self._execute('withdraw',obs,reverse_of=insertion['id'])
            obs=self._observe()
            if obs.get('outside') is not True:raise ValueError('Withdrawal outside position not confirmed')
            self.backend.hold();self._save(status='held_outside',phase='hold',reason='Trial complete; battery never released')
        except Exception as error:
            status='cancelled' if isinstance(error,InterruptedError) else ('recovery_required' if self.state['motion_started'] else 'blocked')
            reason=str(error)
            try:self.backend.hold()
            except Exception as hold_error:reason+='; hold outcome unknown: '+str(hold_error);status='recovery_required'
            self._save(status=status,reason=reason)
        return copy.deepcopy(self.state)
