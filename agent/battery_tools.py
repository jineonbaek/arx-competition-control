"""Opt-in execution stages for the battery workflow.

The existing preview registry stays read-only. A deterministic backend must be
explicitly injected before the fixed lift and adaptive pick stages can execute.
"""
import copy
import hashlib
import json
from .contracts import AgentError, object_schema
from .grasp_tools import WorkflowTools, WORKFLOW_TOOLS
from .tools import ToolRegistry, TEXT
from .mission import Mission
from .battery_pick_plan import plan as plan_battery_pick
from .grasp_refinement import assess
from .storage import atomic_json

EXECUTION_TOOLS=WORKFLOW_TOOLS|{'execute_coarse_alignment','get_execution_status','plan_battery_pick','execute_battery_approach','execute_pick','capture_grasp_scene','verify_hold'}


class BatteryExecutionTools(WorkflowTools):
    observation_limit=3
    schemas={k:v for k,v in WorkflowTools.schemas.items() if k in WORKFLOW_TOOLS}
    schemas.update(capture_grasp_scene=object_schema(),verify_hold=object_schema(),plan_battery_pick=object_schema({'observation_id':TEXT},['observation_id']),
                   execute_battery_approach=object_schema({'plan_id':TEXT},['plan_id']),
                   execute_pick=ToolRegistry.schemas['execute_pick'],
                   execute_coarse_alignment=object_schema({'plan_id':TEXT},['plan_id']),
                   get_execution_status=ToolRegistry.schemas['get_execution_status'])
    descriptions={k:v for k,v in WorkflowTools.descriptions.items() if k in WORKFLOW_TOOLS}
    descriptions.update(capture_grasp_scene='Freeze D405 RGB-D without movement.',verify_hold='Confirm retention from fresh right-gripper feedback and the configured caught-battery range.',plan_battery_pick='Plan insertion along the battery approach axis from this run reviewed D405 preview. Uses only locally configured calibrated TCP and limits; returns TCP pending when absent. No movement or execution permission.',
        execute_battery_approach='Execute a locally verified approach or bounded correction, then require a new D405 scene. Never closes the gripper.',
        execute_pick=ToolRegistry.descriptions['execute_pick'],
        execute_coarse_alignment='Execute only the stored verified coarse plan. '
        'Wait for its existing receipt, then capture a new D405 observation. Never retry motion with a new call ID.',
        get_execution_status=ToolRegistry.descriptions['get_execution_status'])

    def __init__(self,config,store,observations=None,backends=None,*,allowed_tools=None,pick_setup=None,refinement_policy=None):
        if config.services or (config.motion_enabled and backends is None):
            raise ValueError('An explicitly prepared local execution backend is required')
        ToolRegistry.__init__(self,config,store,observations,backends,
                              allowed_tools=EXECUTION_TOOLS if allowed_tools is None else allowed_tools)
        self.pick_setup=copy.deepcopy(pick_setup)
        self.refinement_policy=copy.deepcopy(refinement_policy)
        self.progress=lambda stage:None

    def _wrist_stage(self):
        mission=self.store.state['mission']
        return 'PICK_AT_PREGRASP' if mission.get('approach_plan') else 'COARSE_ALIGNED'

    def _observation_budget_key(self,camera):
        if camera!='d405':return camera
        barrier=self.store.state['mission'].get('evidence_barriers',{}).get(self._wrist_stage(),{})
        return camera+':'+str(barrier.get('completed_at','missing'))

    def _save_pick_plan(self,result):
        path=self.store.path/result['result_file']
        atomic_json(path,result)
        self.store.state['battery_pick_plan_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
        self.store.save()

    def _operation(self,name,arguments,call_id,mission):
        if name in ('execute_battery_approach','execute_pick'):
            mission.require(('PICK_PLANNED',) if name=='execute_pick' else
                            ('PICK_APPROACH_PLANNED','PICK_CORRECTION_PLANNED'))
            plan=mission.data.get('plan') or {}
            if arguments['plan_id']!=plan.get('plan_id'):
                raise AgentError('plan_missing','必须使用本阶段已保存的计划')
            try:
                path=(self.store.path/self.store.state['battery_pick_plan']).resolve()
                if not path.is_relative_to(self.store.path):raise ValueError('invalid path')
                blob=path.read_bytes()
                if (hashlib.sha256(blob).hexdigest()!=self.store.state.get('battery_pick_plan_sha256')
                        or json.loads(blob)!=plan):raise ValueError('plan changed')
                setup_digest=hashlib.sha256(json.dumps(self.pick_setup,sort_keys=True,allow_nan=False).encode()).hexdigest()
                if plan['pick_setup_sha256']!=setup_digest:raise ValueError('TCP changed')
            except (OSError,ValueError,KeyError,TypeError) as error:
                raise AgentError('plan_changed','计划或 TCP 已变化，必须重新观察和规划') from error
            if plan.get('observation_id')!=self._localized('d405'):
                raise AgentError('stale_observation','计划不再对应最新 D405 观察')
            self._post_stage_observation(plan['observation_id'],self._wrist_stage(),'d405')
            if (plan.get('arm')!='right' or any(plan.get(k) is not True for k in
                    ('executable','ik_checked','collision_checked','collision_free','environment_collision_checked'))):
                raise AgentError('plan_not_verified','缺少完整路径、环境碰撞或执行适配证据')
            if name=='execute_pick' and plan.get('refinement',{}).get('status')!='aligned':
                raise AgentError('reobservation_required','必须先到预抓位，再用新画面确认对位')
        elif name=='execute_coarse_alignment':
            mission.require(('COARSE_PLANNED',))
            plan=mission.data.get('plan') or {}
            if arguments['plan_id']!=plan.get('plan_id'):
                raise AgentError('plan_missing','粗对位计划不是本轮已保存的计划')
            if plan.get('observation_id')!=self._localized('d435'):
                raise AgentError('stale_observation','粗对位计划不再对应最新头部观察')
            if (plan.get('arm')!='right'
                    or any(plan.get(k) is not True for k in
                           ('executable','collision_checked','collision_free',
                            'environment_collision_checked'))
                    or plan.get('observer_required') is not False
                    or plan.get('lift',{}).get('fixed') is not True):
                raise AgentError('plan_not_verified','固定升降建议缺少立柱、现场清空或执行适配证据')
        return super()._operation(name,arguments,call_id,mission)

    def _perception(self,name,a,call_id):
        if name not in ('observe_battery','preview_battery_grasp'):
            return super()._dispatch(name,a,call_id)
        camera=a['camera'] if name=='observe_battery' else 'd405'
        attempts=[]
        retryable={'target_not_found','recognition_rejected','depth_rejected',
                   'pose_rejected','handoff_rejected'}
        current=dict(a);attempt_id=call_id
        while True:
            if self.store.cancelled:raise AgentError('cancelled','观察定位已取消')
            result=super()._dispatch(name,current,attempt_id)
            attempts.append(dict(status=result.get('status'),observation_id=result.get('observation_id'),
                                 message=result.get('message','')))
            if self.store.cancelled:raise AgentError('cancelled','观察定位已取消')
            if result.get('status') not in retryable:break
            used=self.store.state.get('battery_observation_attempts',{}).get(self._observation_budget_key(camera),0)
            if used>=self.observation_limit or len(attempts)>=self.observation_limit:break
            self.progress('reobserve_'+camera)
            # External call IDs cannot contain '/', so retries cannot overwrite their artifacts.
            attempt_id=call_id+'/retry_'+str(len(attempts))
            if name=='preview_battery_grasp':
                observed=self._dispatch('observe_battery',{'camera':'d405'},attempt_id)
                if observed.get('status')!='localized':
                    attempts.extend(observed['recovery']['attempts'])
                    result=observed
                    break
                current['observation_id']=observed['observation_id']
        result=dict(result,recovery=dict(attempts=attempts,
            exhausted=result.get('status') in retryable))
        self.store.event('perception_recovery',dict(tool=name,call_id=call_id,**result['recovery']))
        return result

    def _dispatch(self,name,a,call_id):
        state=self.store.state;mission=Mission(state['mission'])
        head=name=='observe_battery' and a['camera']=='d435'
        wrist=name=='observe_battery' and a['camera']=='d405'
        if wrist or name in ('preview_battery_grasp','plan_battery_pick'):
            mission.require(('COARSE_ALIGNED','PICK_AT_PREGRASP','PICK_APPROACH_PLANNED',
                             'PICK_CORRECTION_PLANNED','PICK_PLANNED'))
            mission.data.update(stage=self._wrist_stage(),plan=None)
            state.pop('battery_pick_plan_sha256',None)
            self.store.save()
            if name!='observe_battery':
                self._post_stage_observation(a['observation_id'],self._wrist_stage(),'d405')
        if name=='plan_battery_pick':
            state.pop('battery_pick_plan',None)
            self.store.save()
            if a['observation_id']!=self._localized('d405'):
                raise AgentError('stale_observation','必须使用本轮最新 D405 定位')
            try:
                result=plan_battery_pick(self,a['observation_id'],call_id,self.pick_setup)
                result['plan_id']='pick_'+hashlib.sha256((state['run_id']+':'+call_id).encode()).hexdigest()[:24]
                if result['status']!='tcp_pending':
                    previous=mission.data.get('approach_plan')
                    if previous:
                        result['refinement']=(assess(result,previous,self.refinement_policy,
                            corrections=mission.data.get('pick_corrections',0),
                            total_mm=mission.data.get('pick_correction_mm',0)) if self.refinement_policy is not None
                            else dict(status='policy_pending',executable=False))
                        next_stage={'aligned':'PICK_PLANNED','correction_required':'PICK_CORRECTION_PLANNED'}.get(
                            result['refinement']['status'],'PICK_AT_PREGRASP')
                    else:next_stage='PICK_APPROACH_PLANNED'
                    mission.data.update(stage=next_stage,plan=copy.deepcopy(result) if next_stage!='PICK_AT_PREGRASP' else None)
                self._save_pick_plan(result)
                return result
            except (OSError,ValueError,KeyError,TypeError) as error:
                state.pop('battery_pick_plan',None);state.pop('battery_pick_plan_sha256',None)
                mission.data.update(stage=self._wrist_stage(),plan=None)
                raise AgentError('pick_plan_failed','抓取规划证据或参数无效：'+str(error)[:180]) from error
        if head:
            mission.require(('IDLE','HEAD_LOCALIZED','COARSE_PLANNED'))
            mission.data.update(stage='IDLE',plan=None)
            self.store.save()
        elif name=='plan_coarse_alignment':
            mission.require(('HEAD_LOCALIZED','COARSE_PLANNED'))
            mission.data.update(stage='HEAD_LOCALIZED',plan=None)
            self.store.save()
        result=self._perception(name,a,call_id)
        if head and result.get('status')=='localized':
            mission.data['stage']='HEAD_LOCALIZED'
        elif name=='plan_coarse_alignment':
            ident=hashlib.sha256((state['run_id']+':'+call_id).encode()).hexdigest()[:24]
            result=dict(result,plan_id='coarse_'+ident,arm='right')
            mission.data.update(stage='COARSE_PLANNED',plan=copy.deepcopy(result))
        elif wrist:
            try:self._post_stage_observation(result['observation_id'],self._wrist_stage(),'d405')
            except AgentError:
                state.get('latest_battery_observations',{}).pop('d405',None)
                state.pop('grasp_preview',None)
                raise
        return result
