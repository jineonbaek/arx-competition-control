"""Read-only Agent tools composing recognition, pose and geometric preview."""
from pathlib import Path
import hashlib
from .contracts import AgentError,object_schema
from .tools import ToolRegistry,TEXT
from . import grasp_scene
from .grasp_pipeline import pipeline
from .battery_observation import observe
from .coarse_alignment import plan as plan_coarse

PREVIEW_TOOLS={"get_system_status","observe_scene","read_observation","capture_grasp_scene","preview_battery_grasp","observe_battery","plan_coarse_alignment"}

class GraspTools(ToolRegistry):
    require_head_evidence=False
    schemas={k:v for k,v in ToolRegistry.schemas.items() if k in PREVIEW_TOOLS}
    schemas.update(plan_coarse_alignment=object_schema({"observation_id":TEXT},["observation_id"]),
        observe_battery=object_schema({"camera":{"type":"string","enum":["d435","d405"]}},["camera"]),
        capture_grasp_scene=object_schema(),
        preview_battery_grasp=object_schema({"observation_id":TEXT,
            "clearance_mm":{"type":"number","minimum":1,"maximum":200}},["observation_id","clearance_mm"]))
    descriptions={k:v for k,v in ToolRegistry.descriptions.items() if k in PREVIEW_TOOLS}
    descriptions.update(plan_coarse_alignment="Use this run's localized D435 evidence to identify the target and emit the configured fixed coarse lift target plus fixed right-arm observation pose. X/Y stay near the initial wrist position while Z starts at the verified observation height. The surface is not the grip center; the proposal reports both-arm column checks and remains non-executable until local review. TCP remains empty. No movement.",
        observe_battery="Freeze selected RGB-D camera, recognize one shape-defined battery and estimate its visible surface in base_link. D435 is coarse CAD localization; D405 is wrist-calibrated. Surface point is not a grip point. Returns explicit not-found/rejection and evidence ID. No robot motion.",
        capture_grasp_scene="Freeze paired D405 RGB-D and snapshot-time base_link transform using the existing owner. Temporary depth lease; no robot motion.",
        preview_battery_grasp="On one frozen D405 scene: reviewed battery recognition, FoundationPose, cloud evidence and +X constant-YZ contact-point path. TCP pending; no wrist targets or commands.")
    def __init__(self,config,store,*args,**kwargs):
        if config.motion_enabled or config.services:raise ValueError("Preview tools cannot have physical backends")
        super().__init__(config,store,*args,**kwargs)
        self.progress=lambda stage:None
    def _dispatch(self,name,a,call_id):
        try:
            if name=="plan_coarse_alignment":return plan_coarse(self,a["observation_id"],call_id)
            if name=="observe_battery":return observe(self,a["camera"],call_id)
            if name=="capture_grasp_scene":return grasp_scene.capture(self)
            if name=="preview_battery_grasp":
                for key in ("grasp_preview","grasp_preview_sha256","battery_pick_plan"):
                    self.store.state.pop(key,None)
                self.store.save()
                result=pipeline(self.store,a["observation_id"],a["clearance_mm"],
                    self.store.path/"previews"/call_id,self.progress,
                    head_observation_id=self._localized('d435') if self.require_head_evidence else None)
                relative=Path("previews")/call_id/"result.json"
                digest=hashlib.sha256((self.store.path/relative).read_bytes()).hexdigest()
                self.store.state.update(grasp_preview=str(relative),grasp_preview_sha256=digest)
                return {"observation_id":a["observation_id"],"status":result["status"],"message":result["message"],
                    "tcp_status":"pending","executable":False,"result_file":self.store.state["grasp_preview"],
                    "quality":result.get("quality"),"camera_handoff":result.get("camera_handoff"),"depth":result.get("geometry",{}).get("depth"),"path":result.get("geometry",{}).get("path")}
            return super()._dispatch(name,a,call_id)
        except AgentError:raise
        except (OSError,ValueError,KeyError) as e:
            raise AgentError("preview_failed","预览证据或环境无效："+str(e)[:180]) from e


WORKFLOW_TOOLS={'get_system_status','read_observation','observe_battery','plan_coarse_alignment','preview_battery_grasp'}

class WorkflowTools(GraspTools):
    """Enforce prerequisite evidence even if the model asks for stages out of order."""
    require_head_evidence=True
    observation_limit=2
    def _localized(self,camera):
        import json
        ident=self.store.state.get('latest_battery_observations',{}).get(camera)
        relative=self.store.state.get('battery_observations',{}).get(ident)
        if not relative:raise AgentError('stage_not_ready',camera+' 尚无本轮定位证据')
        path=(self.store.path/relative).resolve()
        if not path.is_relative_to(self.store.path.resolve()):raise AgentError('invalid_run','证据路径无效')
        try:evidence=json.loads(path.read_text())
        except (OSError,ValueError) as error:raise AgentError('invalid_run','定位证据不可读取') from error
        if evidence.get('status')!='localized' or evidence.get('camera')!=camera or evidence.get('observation_id')!=ident:
            raise AgentError('stage_not_ready',camera+' 未定位成功，不能继续下一阶段')
        return ident

    def _observation_budget_key(self,camera):
        return camera

    def _observation_limit(self,camera):
        return self.observation_limit

    def _dispatch(self,name,a,call_id):
        if name=='observe_battery' and a['camera']=='d405':
            self._localized('d435')
            if not self.store.state.get('coarse_alignment'):
                raise AgentError('stage_not_ready','先规划 D435 粗对位；本模式仍不会执行动作')
        if name=='plan_coarse_alignment' and self._localized('d435')!=a['observation_id']:
            raise AgentError('stale_observation','必须使用本轮最新 D435 定位')
        if name=='preview_battery_grasp' and self._localized('d405')!=a['observation_id']:
            raise AgentError('stale_observation','必须使用本轮最新 D405 定位')
        if name=='observe_battery':
            counts=self.store.state.setdefault('battery_observation_attempts',{})
            camera=self._observation_budget_key(a['camera'])
            limit=self._observation_limit(a['camera'])
            if counts.get(camera,0)>=limit:
                raise AgentError('observation_budget',f'本阶段该相机已观察 {limit} 次，请查看原因后开始新任务')
            counts[camera]=counts.get(camera,0)+1
            self.store.save()
        return super()._dispatch(name,a,call_id)
