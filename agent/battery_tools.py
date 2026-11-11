"""Opt-in execution stages for the battery workflow; not registered by the live UI.

The existing preview registry stays read-only. A deterministic backend must be
explicitly injected; the current nominal coarse planner cannot grant execution.
"""
import copy
import hashlib
from .contracts import AgentError, object_schema
from .grasp_tools import WorkflowTools, WORKFLOW_TOOLS
from .tools import ToolRegistry, TEXT
from .mission import Mission

EXECUTION_TOOLS=WORKFLOW_TOOLS|{'execute_coarse_alignment','get_execution_status'}


class BatteryExecutionTools(WorkflowTools):
    schemas={k:v for k,v in WorkflowTools.schemas.items() if k in WORKFLOW_TOOLS}
    schemas.update(execute_coarse_alignment=object_schema({'plan_id':TEXT},['plan_id']),
                   get_execution_status=ToolRegistry.schemas['get_execution_status'])
    descriptions={k:v for k,v in WorkflowTools.descriptions.items() if k in WORKFLOW_TOOLS}
    descriptions.update(execute_coarse_alignment='Execute only the stored verified coarse plan. '
        'Wait for its existing receipt, then capture a new D405 observation. Never retry motion with a new call ID.',
        get_execution_status=ToolRegistry.descriptions['get_execution_status'])

    def __init__(self,config,store,observations=None,backends=None,*,allowed_tools=None):
        if config.services or (config.motion_enabled and backends is None):
            raise ValueError('An explicitly prepared local execution backend is required')
        ToolRegistry.__init__(self,config,store,observations,backends,
                              allowed_tools=EXECUTION_TOOLS if allowed_tools is None else allowed_tools)
        self.progress=lambda stage:None

    def _operation(self,name,arguments,call_id,mission):
        if name=='execute_coarse_alignment':
            mission.require(('COARSE_PLANNED',))
            plan=mission.data.get('plan') or {}
            if arguments['plan_id']!=plan.get('plan_id'):
                raise AgentError('plan_missing','粗对位计划不是本轮已保存的计划')
            if plan.get('observation_id')!=self._localized('d435'):
                raise AgentError('stale_observation','粗对位计划不再对应最新头部观察')
            if plan.get('arm')!='right' or any(plan.get(k) is not True for k in
                    ('executable','ik_checked','collision_checked','collision_free')):
                raise AgentError('plan_not_verified','粗对位仍是建议，缺少可执行路径或完整碰撞检查')
        return super()._operation(name,arguments,call_id,mission)

    def _dispatch(self,name,a,call_id):
        state=self.store.state;mission=Mission(state['mission'])
        head=name=='observe_battery' and a['camera']=='d435'
        wrist=name=='observe_battery' and a['camera']=='d405'
        if head:
            mission.require(('IDLE','HEAD_LOCALIZED','COARSE_PLANNED'))
            mission.data.update(stage='IDLE',plan=None)
            self.store.save()
        elif name=='plan_coarse_alignment':
            mission.require(('HEAD_LOCALIZED','COARSE_PLANNED'))
            mission.data.update(stage='HEAD_LOCALIZED',plan=None)
            self.store.save()
        elif wrist or name=='preview_battery_grasp':
            mission.require(('COARSE_ALIGNED',))
            if name=='preview_battery_grasp':
                self._post_stage_observation(a['observation_id'],'COARSE_ALIGNED','d405')
        result=super()._dispatch(name,a,call_id)
        if head and result.get('status')=='localized':
            mission.data['stage']='HEAD_LOCALIZED'
        elif name=='plan_coarse_alignment':
            ident=hashlib.sha256((state['run_id']+':'+call_id).encode()).hexdigest()[:24]
            result=dict(result,plan_id='coarse_'+ident,arm='right')
            mission.data.update(stage='COARSE_PLANNED',plan=copy.deepcopy(result))
        elif wrist:
            try:self._post_stage_observation(result['observation_id'],'COARSE_ALIGNED','d405')
            except AgentError:
                state.get('latest_battery_observations',{}).pop('d405',None)
                state.pop('grasp_preview',None)
                raise
        return result
