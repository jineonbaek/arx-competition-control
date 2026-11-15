"""Placement assembly reuses the automatic grasp task owner; opt-in and inert."""
import json
import hashlib
import math
from pathlib import Path
from battery_task import BatteryTask
from agent.placement_backend import PlacementBackend
from agent.placement_flow import PlacementFlow

class PlacementTask(BatteryTask):
    def __init__(self,*args,execution_enabled=False,**kwargs):
        self.execution_enabled=execution_enabled
        super().__init__(*args,**kwargs)

    def _unresolved(self):
        root=self.root
        try:resolved=json.loads((root/'recovery.json').read_text()).get('resolved',{})
        except (OSError,ValueError,AttributeError):resolved={}
        active=self.active
        running=active['run_id'] if active and active.get('status')=='running' else None
        blocked=[]
        for p in root.glob('run-*/state.json'):
            if p.parent.name==running:continue
            try:
                raw=p.read_bytes();data=json.loads(raw);flow=data.get('placement_flow',{})
                physical=flow.get('motion_started') is True or flow.get('phase') in ('preplace','correct','reobserve','descend','release','withdraw','verify')
                finished=flow.get('status')=='completed' and flow.get('placement_confirmed') is True and not flow.get('inflight')
                if not physical or finished:continue
                receipt=resolved.get(p.parent.name,{})
                if (isinstance(receipt,dict) and receipt.get('state_sha256')==hashlib.sha256(raw).hexdigest()
                    and receipt.get('platform_stopped') is True and receipt.get('holding_state_checked') is True
                    and receipt.get('environment_clear') is True and isinstance(receipt.get('note'),str) and receipt['note'].strip()):continue
                blocked.append(p.parent.name)
            except (OSError,ValueError,TypeError,AttributeError):blocked.append(p.parent.name)
        return blocked

    def _start(self,*args,**kwargs):
        with self.lock:
            unresolved=self._unresolved()
            if unresolved:raise ValueError('存在未确认放置任务，先人工恢复：'+', '.join(unresolved))
            return super()._start(*args,**kwargs)

    def action(self,client,data):
        if not isinstance(data,dict):raise ValueError('无效的放置请求')
        if data.get('action')=='start':
            if data.get('holding_confirmed') is not True:raise ValueError('请确认右夹爪仍可靠夹持电池')
            data={k:v for k,v in data.items() if k!='holding_confirmed'}
        if data.get('action')=='start' and not self.execution_enabled:
            raise ValueError('放置执行未启用，当前可进行红圈识别与规划预览')
        if set(data)&{'lift_target_rad','observer_z_mm'}:raise ValueError('持物放置保持升降，不能复用抓取升降设置')
        return super().action(client,data)

    def snapshot(self,client):
        with self.lock:
            result={**super().snapshot(client),'execution_enabled':self.execution_enabled,'side':'left'}
            unresolved=self._unresolved()
            if unresolved:result.update(available=False,recovery_required=True,unresolved_runs=unresolved,reason='未确认的放置动作需人工恢复，禁止重新开始')
            return result


def make_placement_task(controller,preview,policy_path,depth_control,root):
    if policy_path is None:return None
    settings=json.loads(Path(policy_path).read_text())
    required={'execution_enabled','placement_geometry_verified','asset_root','grasp_policy','geometry','execution_observation_max_age_s'}
    if set(settings)!=required or type(settings['execution_enabled']) is not bool or type(settings['placement_geometry_verified']) is not bool:
        raise ValueError('放置配置字段或执行开关无效')
    value=settings['execution_observation_max_age_s']
    if type(value) not in (int,float) or not math.isfinite(value) or not 1<=value<=120:raise ValueError('执行观测有效期须为1–120秒')
    from agent.placement_geometry import policy
    policy(settings['geometry'])
    enabled=settings['execution_enabled'] and settings['placement_geometry_verified']
    # Disabled examples stay preview-only and do not need to load site-owned execution limits.
    pick_policy=None
    if enabled:
        from agent.execution_policy import load
        pick_policy=load(settings['grasp_policy'],gripper_limits=controller.config['arms']['right']['gripper_limits_raw'])
    def factory(store,ctrl,token,environment_clear):
        if not enabled:raise ValueError('放置执行未启用')
        return PlacementFlow(store,PlacementBackend(store,ctrl,token,settings,pick_policy,preview,environment_clear=environment_clear))
    return PlacementTask(Path(root)/'agent/placement-runs',controller,factory,depth_control=depth_control,execution_enabled=enabled)
