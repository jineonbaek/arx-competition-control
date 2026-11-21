"""Deterministic battery sequence through the existing tools; no LLM planner."""
import copy
import hashlib
import json
import math
import time
from threading import Lock
from .fast_pick import fast_requested, stopped_for_fast_pick
from .contracts import AgentError

COMMON_CHECKS=('collision_checked','collision_free','environment_collision_checked')
PICK_CHECKS=('ik_checked',)+COMMON_CHECKS

def plan_digest(plan):
    return hashlib.sha256(json.dumps(plan,sort_keys=True,allow_nan=False).encode()).hexdigest()

class BatteryFlow:
    def __init__(self,tools,*,review_plan,clearance_mm=50.,receipt_timeout_s=240.,vision_mode='d405'):
        if (not callable(review_plan) or type(clearance_mm) not in (int,float)
                or not math.isfinite(clearance_mm) or not 1<=clearance_mm<=200
                or type(receipt_timeout_s) not in (int,float) or not math.isfinite(receipt_timeout_s)
                or not 0<receipt_timeout_s<=3600 or vision_mode not in ('d405','d435','plan_f','fixed')):
            raise ValueError('Local reviewer, known vision mode and bounded flow settings required')
        self.tools=tools;self.store=tools.store;self.review_plan=review_plan
        self.receipt_timeout_s=receipt_timeout_s
        if not hasattr(self.store,"battery_flow_lock"):self.store.battery_flow_lock=Lock()
        self.flow=self.store.state.setdefault('battery_flow',dict(status='running',
            phase='fixed_coarse_plan' if vision_mode=='fixed' else 'head',
            index=0,polls=0,clearance_mm=clearance_mm,vision_mode=vision_mode,grasp_confirmed=False))
        if self.flow['clearance_mm']!=clearance_mm:raise ValueError('Cannot change a persisted flow')
        if self.flow.get('vision_mode','d405')!=vision_mode:raise ValueError('Cannot change a persisted vision mode')
        self.vision_mode=vision_mode
        self.store.save()

    def _finish(self,status,reason=''):
        if status=='completed' and self.store.cancelled:status,reason='cancelled','Cancelled during completion'
        self.flow.update(status=status,reason=reason,grasp_confirmed=status=='completed')
        self.store.save();self.store.event('battery_flow',copy.deepcopy(self.flow))
        return copy.deepcopy(self.flow)

    def _next(self,phase):
        self.flow.update(phase=phase,index=self.flow['index']+1)
        self.flow.pop('pending_since',None);self.flow.pop('command',None);self.store.save()

    def _cache_target(self,plan):
        if self.vision_mode!='plan_f' or fast_requested(self.store) or plan.get('status')!='kinematic_preview':return
        source='B' if self.store.state['mission'].get('approach_plan') else 'A'
        cache=self.flow.setdefault('target_cache',{})
        if source not in cache:
            cache[source]=dict(plan=copy.deepcopy(plan),preview_file=self.store.state['grasp_preview'])
            self.store.save()
            self.store.event('target_cache_saved',dict(source=source,observation_id=plan['observation_id'],
                plan_id=plan['plan_id'],name='保存缓存目标 '+source+'，观察 '+plan['observation_id']))

    def _visual_call(self,name,args,success):
        try:
            data=self._call(name,args)
            if success and data.get('status')!=success:
                raise AgentError('vision_failed',data.get('message') or data.get('status','vision_failed'))
            return data
        except Exception as error:
            if (self.vision_mode!='plan_f' or fast_requested(self.store) or self.store.cancelled
                    or self.store.state.get('pending') or not self.store.state['mission'].get('approach_plan')
                    or not self.flow.get('target_cache')
                    or isinstance(error,AgentError) and error.code not in
                       ('vision_failed','preview_failed','pick_plan_failed','observation_budget','stage_not_ready',
                        'post_action_evidence_required','stale_observation','stale_snapshot','scene_changed')):
                raise
            self.flow['fallback_reason']=str(error)[:300]
            self.store.event('target_cache_fallback',dict(phase=self.flow['phase'],reason=str(error)[:300],
                name='视觉失败，转缓存兜底：'+str(error)[:300]))
            self._next('cached_replan')
            return None

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
                last_receipt=(self.store.state.get('pending') or {}).get('last_receipt') or {}
                last_data=last_receipt.get('data',{})
                if (name=='stow_for_carry' and last_receipt.get('status')=='running'
                        and last_data.get('extract_stage')=='backing_chassis'
                        and last_data.get('waiting_for_pose') is True):
                    self.flow['pending_since']=time.time()
                if time.time()-self.flow['pending_since']>self.receipt_timeout_s:
                    raise AgentError('execution_timeout','Execution receipt deadline exceeded')
                self.flow['polls']+=1;self.store.save()
                result=self.tools.execute('get_execution_status',{},'fixed_poll_'+str(self.flow['polls']))
                receipt=(self.store.state.get('pending') or {}).get('last_receipt',{})
                data=receipt.get('data',{})
                if (data.get('recovery_required') or data.get('reason')=='recovery_required'
                        or any(data.get(k) in ('recovery_required','receipt_unavailable','execution_unknown')
                               for k in ('coarse_stage','approach_stage','pick_stage'))):
                    raise AgentError('execution_unknown',data.get('failure_detail')
                                     or 'Existing motion receipt requires recovery; will not replay')
                if self.store.state.get('pending'):time.sleep(.1)
        if self.store.cancelled:raise AgentError('cancelled','Task cancelled during tool completion')
        if not result['ok']:
            raise AgentError(result['error']['code'],result['error']['message'])
        return result['data']

    def _review(self,operation):
        plan=self.store.state['mission']['plan']
        digest=plan_digest(plan)
        if self.flow.get('reviewed_plan')==digest:return
        evidence=self.review_plan(operation,copy.deepcopy(plan))
        required=PICK_CHECKS
        if (not isinstance(evidence,dict) or evidence.get('plan_sha256')!=digest
                or any(evidence.get(k) is not True for k in required)):
            raise AgentError('plan_not_verified','Local path review incomplete or rejected')
        plan.update({k:True for k in required},executable=True)
        plan['execution_review']=copy.deepcopy(evidence)
        if operation!='execute_coarse_alignment':self.tools._save_pick_plan(plan)
        self.flow['reviewed_plan']=plan_digest(plan)
        self.store.save();self.store.event('execution_review',dict(operation=operation,**evidence))

    def run(self):
        if not self.store.battery_flow_lock.acquire(blocking=False):
            return dict(status='running',reason='Flow already running',grasp_confirmed=False)
        try:return self._run()
        finally:
            try:
                session=getattr(self.store,'perception_session',None)
                if session is not None:session.close()
            finally:self.store.battery_flow_lock.release()

    def _run(self):
        if self.flow['status']!='running':return copy.deepcopy(self.flow)
        try:
            while (self.flow['index']<self.tools.config.max_tool_calls
                   or self.store.state['mission'].get('approach_plan')):
                if self.store.cancelled:raise AgentError('cancelled','Task cancelled')
                phase=self.flow['phase']
                if phase=='fixed_coarse_plan':
                    plan=self.tools.fixed_coarse_plan()
                    self.store.state['mission'].update(stage='COARSE_PLANNED',plan=copy.deepcopy(plan),
                        target={'target_id':'plan_c_fixed','camera':'none'})
                    self.store.save();self._next('coarse_execute')
                elif phase=='fixed_pick_plan':
                    plan=self.tools.fixed_pick_plan()
                    self.flow['plan_id']=plan['plan_id']
                    self.store.state['mission'].update(stage='PICK_PLANNED',plan=copy.deepcopy(plan))
                    self.store.save();self._next('insert')
                elif phase=='head':
                    if fast_requested(self.store) and not stopped_for_fast_pick(self.store):
                        raise AgentError('dock_incomplete','等待本轮底盘贴靠停车回执')
                    camera='d405' if self.vision_mode=='plan_f' else 'd435'
                    data=self._call('observe_battery',{'camera':camera})
                    if data.get('status')!='localized':return self._finish('needs_attention',data.get('status','head_failed'))
                    self.flow['head']=data['observation_id']
                    self.store.state['mission']['target']=dict(target_id=data['observation_id'],camera=camera,observation_id=data['observation_id'])
                    if self.vision_mode=='plan_f':
                        self.flow['wrist']=data['observation_id']
                        self.store.state['mission']['stage']='COARSE_ALIGNED'
                        self.store.save();self._next('preview')
                    else:self._next('preview' if self.vision_mode=='d435' else 'coarse_plan')
                elif phase=='coarse_plan':
                    self._call('plan_coarse_alignment',{'observation_id':self.flow['head']});self._next('coarse_execute')
                elif phase in ('coarse_execute','approach_execute','insert'):
                    operation={'coarse_execute':'execute_coarse_alignment','approach_execute':'execute_battery_approach','insert':'execute_pick'}[phase]
                    # After restart, resolve the existing request before any new review or action.
                    if not self.store.state.get('pending') and not self.flow.get('command'):self._review(operation)
                    plan=self.store.state['mission'].get('plan') or {}
                    self._call(operation,{'plan_id':plan.get('plan_id',self.flow.get('plan_id'))})
                    self._next('verify_hold' if phase=='insert' else
                               'fixed_pick_plan' if self.vision_mode=='fixed' else
                               'pick_plan' if self.vision_mode=='d435' and phase=='coarse_execute' else 'wrist')
                elif phase=='wrist':
                    data=self._visual_call('observe_battery',{'camera':'d405'},'localized')
                    if data is None:continue
                    self.flow['wrist']=data['observation_id'];self._next('preview')
                elif phase=='preview':
                    observation=self.flow['head'] if self.vision_mode=='d435' else self.flow['wrist']
                    data=self._visual_call('preview_battery_grasp',dict(observation_id=observation,clearance_mm=self.flow['clearance_mm']),'preview_complete')
                    if data is None:continue
                    self.flow['wrist']=data['observation_id']
                    self._next('coarse_plan' if self.vision_mode=='d435' else 'pick_plan')
                elif phase=='pick_plan':
                    data=self._visual_call('plan_battery_pick',{'observation_id':self.flow['wrist']},None)
                    if data is None:continue
                    self._cache_target(data)
                    self.flow['plan_id']=data.get('plan_id')
                    stage=self.store.state['mission']['stage']
                    if stage=='PICK_PLANNED':self._next('insert')
                    elif stage in ('PICK_APPROACH_PLANNED','PICK_CORRECTION_PLANNED'):self._next('approach_execute')
                    elif data.get('refinement',{}).get('status')=='reacquire_required':self._next('wrist')
                    else:return self._finish('needs_attention',data.get('status','planning_incomplete'))
                elif phase=='cached_replan':
                    attempt=self.flow.get('fallback_attempts',0)+1
                    if attempt>3:
                        return self._finish('needs_attention','缓存兜底三次规划失败：'+self.flow.get('fallback_error','')+
                                            '；原视觉错误：'+self.flow.get('fallback_reason',''))
                    source='B' if attempt==1 and 'B' in self.flow['target_cache'] else 'A'
                    self.flow['fallback_attempts']=attempt;self.store.save()
                    self.store.event('target_cache_retry',dict(attempt=attempt,limit=3,source=source,status='planning',
                        name=f'缓存兜底 {attempt}/3：使用 {source} 重新规划'))
                    try:
                        data=self.tools.replan_cached_target(source,'cached_'+str(attempt))
                        self._review('execute_pick')
                    except Exception as error:
                        self.flow['fallback_error']=str(error)[:300]
                        self.store.event('target_cache_retry',dict(attempt=attempt,source=source,status='failed',reason=str(error)[:300],
                            name=f'缓存兜底 {attempt}/3，{source} 规划失败：'+str(error)[:300]))
                        self._next('cached_replan');continue
                    self.flow['plan_id']=data['plan_id']
                    self.store.event('target_cache_retry',dict(attempt=attempt,source=source,status='planned',plan_id=data['plan_id'],
                        name=f'缓存兜底 {attempt}/3，{source} 规划通过，进入原插入流程'))
                    self._next('insert')
                elif phase=='hold_capture':
                    # Compatibility for a run persisted by the earlier visual-verification flow.
                    self._next('verify_hold')
                elif phase=='verify_hold':
                    if not self.tools.backends.available('verify_hold'):
                        return self._finish('awaiting_hold_verification','Gripper feedback range is not configured')
                    self._call('verify_hold',{})
                    if self.store.state['mission']['stage']!='HOLD_CONFIRMED':
                        return self._finish('awaiting_hold_verification','Retention not confirmed')
                    self._next('extract')
                elif phase=='extract':
                    self._call('stow_for_carry',{'plan_id':self.store.state['mission']['plan']['plan_id']})
                    if fast_requested(self.store):
                        return self._finish('completed')
                    self._next('carry')
                elif phase=='carry':
                    self._call('return_to_carry',{'plan_id':self.store.state['mission']['plan']['plan_id']})
                    return self._finish('completed')
                else:raise AgentError('invalid_flow','Unknown fixed-flow phase')
            return self._finish('needs_attention','Fixed flow call budget exhausted')
        except Exception as error:
            if self.store.state.get('pending'):
                try:self.tools.backends.cancel_and_wait(2.)
                except Exception:pass
            return self._finish('cancelled' if self.store.cancelled else 'needs_attention',str(error)[:300])
