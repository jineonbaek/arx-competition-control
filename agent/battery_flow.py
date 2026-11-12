"""Deterministic battery sequence through the existing tools; no LLM planner."""
import copy
import hashlib
import json
import math
import time
from threading import Lock
from .contracts import AgentError

CHECKS=('ik_checked','collision_checked','collision_free','environment_collision_checked')

def plan_digest(plan):
    return hashlib.sha256(json.dumps(plan,sort_keys=True,allow_nan=False).encode()).hexdigest()

class BatteryFlow:
    def __init__(self,tools,*,review_plan,clearance_mm=50.,receipt_timeout_s=240.):
        if (not callable(review_plan) or type(clearance_mm) not in (int,float)
                or not math.isfinite(clearance_mm) or not 1<=clearance_mm<=200
                or type(receipt_timeout_s) not in (int,float) or not math.isfinite(receipt_timeout_s)
                or not 0<receipt_timeout_s<=3600):raise ValueError('Local reviewer and bounded flow settings required')
        self.tools=tools;self.store=tools.store;self.review_plan=review_plan
        self.receipt_timeout_s=receipt_timeout_s
        if not hasattr(self.store,"battery_flow_lock"):self.store.battery_flow_lock=Lock()
        self.flow=self.store.state.setdefault('battery_flow',dict(status='running',phase='head',
            index=0,polls=0,clearance_mm=clearance_mm,grasp_confirmed=False))
        if self.flow['clearance_mm']!=clearance_mm:raise ValueError('Cannot change a persisted flow')
        self.store.save()

    def _finish(self,status,reason=''):
        self.flow.update(status=status,reason=reason,grasp_confirmed=status=='completed')
        self.store.save();self.store.event('battery_flow',copy.deepcopy(self.flow))
        return copy.deepcopy(self.flow)

    def _next(self,phase):
        self.flow.update(phase=phase,index=self.flow['index']+1)
        self.flow.pop('pending_since',None);self.flow.pop('command',None);self.store.save()

    def _call(self,name,args):
        if self.store.cancelled:raise AgentError('cancelled','Task cancelled; no automatic resume')
        command=self.flow.setdefault('command',dict(name=name,arguments=copy.deepcopy(args)))
        if command['name']!=name:raise AgentError('invalid_flow','Persisted tool phase mismatch')
        self.store.save()
        result=self.tools.execute(name,command['arguments'],'fixed_'+str(self.flow['index']))
        if self.store.state.get('pending'):
            self.flow.setdefault('pending_since',time.time());self.store.save()
            while self.store.state.get('pending'):
                if self.store.cancelled:raise AgentError('cancelled','Task cancelled during execution')
                if time.time()-self.flow['pending_since']>self.receipt_timeout_s or self.flow['polls']>=7200:
                    raise AgentError('execution_timeout','Execution receipt deadline exceeded')
                self.flow['polls']+=1;self.store.save()
                result=self.tools.execute('get_execution_status',{},'fixed_poll_'+str(self.flow['polls']))
                receipt=(self.store.state.get('pending') or {}).get('last_receipt',{})
                data=receipt.get('data',{})
                if (data.get('recovery_required') or data.get('reason')=='recovery_required'
                        or any(data.get(k) in ('recovery_required','receipt_unavailable','execution_unknown')
                               for k in ('coarse_stage','approach_stage','pick_stage'))):
                    raise AgentError('execution_unknown','Existing motion receipt requires recovery; will not replay')
                if self.store.state.get('pending'):time.sleep(.1)
        if not result['ok']:
            raise AgentError(result['error']['code'],result['error']['message'])
        return result['data']

    def _review(self,operation):
        plan=self.store.state['mission']['plan']
        digest=plan_digest(plan)
        if self.flow.get('reviewed_plan')==digest:return
        evidence=self.review_plan(operation,copy.deepcopy(plan))
        if (not isinstance(evidence,dict) or evidence.get('plan_sha256')!=digest
                or any(evidence.get(k) is not True for k in CHECKS)):
            raise AgentError('plan_not_verified','Local path review incomplete or rejected')
        plan.update({k:True for k in CHECKS},executable=True)
        plan['execution_review']=copy.deepcopy(evidence)
        if operation!='execute_coarse_alignment':self.tools._save_pick_plan(plan)
        self.flow['reviewed_plan']=plan_digest(plan)
        self.store.save();self.store.event('execution_review',dict(operation=operation,**evidence))

    def run(self):
        if not self.store.battery_flow_lock.acquire(blocking=False):
            return dict(status='running',reason='Flow already running',grasp_confirmed=False)
        try:return self._run()
        finally:self.store.battery_flow_lock.release()

    def _run(self):
        if self.flow['status']!='running':return copy.deepcopy(self.flow)
        try:
            while self.flow['index']<self.tools.config.max_tool_calls:
                if self.store.cancelled:raise AgentError('cancelled','Task cancelled')
                phase=self.flow['phase']
                if phase=='head':
                    data=self._call('observe_battery',{'camera':'d435'})
                    if data.get('status')!='localized':return self._finish('needs_attention',data.get('status','head_failed'))
                    self.flow['head']=data['observation_id']
                    self.store.state['mission']['target']=dict(target_id=data['observation_id'],camera='d435',observation_id=data['observation_id'])
                    self._next('coarse_plan')
                elif phase=='coarse_plan':
                    self._call('plan_coarse_alignment',{'observation_id':self.flow['head']});self._next('coarse_execute')
                elif phase in ('coarse_execute','approach_execute','insert'):
                    operation={'coarse_execute':'execute_coarse_alignment','approach_execute':'execute_battery_approach','insert':'execute_pick'}[phase]
                    # After restart, resolve the existing request before any new review or action.
                    if not self.store.state.get('pending') and not self.flow.get('command'):self._review(operation)
                    plan=self.store.state['mission'].get('plan') or {}
                    self._call(operation,{'plan_id':plan.get('plan_id',self.flow.get('plan_id'))})
                    self._next('hold_capture' if phase=='insert' else 'wrist')
                elif phase=='wrist':
                    data=self._call('observe_battery',{'camera':'d405'})
                    if data.get('status')!='localized':return self._finish('needs_attention',data.get('status','wrist_failed'))
                    self.flow['wrist']=data['observation_id'];self._next('preview')
                elif phase=='preview':
                    data=self._call('preview_battery_grasp',dict(observation_id=self.flow['wrist'],clearance_mm=self.flow['clearance_mm']))
                    if data.get('status')!='preview_complete':return self._finish('needs_attention',data.get('status','preview_failed'))
                    self.flow['wrist']=data['observation_id'];self._next('pick_plan')
                elif phase=='pick_plan':
                    data=self._call('plan_battery_pick',{'observation_id':self.flow['wrist']})
                    self.flow['plan_id']=data.get('plan_id')
                    stage=self.store.state['mission']['stage']
                    if stage=='PICK_PLANNED':self._next('insert')
                    elif stage in ('PICK_APPROACH_PLANNED','PICK_CORRECTION_PLANNED'):self._next('approach_execute')
                    elif data.get('refinement',{}).get('status')=='reacquire_required':self._next('wrist')
                    else:return self._finish('needs_attention',data.get('status','planning_incomplete'))
                elif phase=='hold_capture':
                    if not self.tools.backends.available('verify_hold'):
                        return self._finish('awaiting_hold_verification','Closed gripper is not proof of retention; verifier unavailable')
                    data=self._call('capture_grasp_scene',{});self.flow['hold']=data['observation_id'];self._next('verify_hold')
                elif phase=='verify_hold':
                    self._call('verify_hold',{'observation_id':self.flow['hold']})
                    if self.store.state['mission']['stage']!='HOLD_CONFIRMED':
                        return self._finish('awaiting_hold_verification','Retention not confirmed')
                    return self._finish('completed')
                else:raise AgentError('invalid_flow','Unknown fixed-flow phase')
            return self._finish('needs_attention','Fixed flow call budget exhausted')
        except Exception as error:
            if self.store.state.get('pending'):
                try:self.tools.backends.cancel_and_wait(2.)
                except Exception:pass
            return self._finish('cancelled' if self.store.cancelled else 'needs_attention',str(error)[:300])
