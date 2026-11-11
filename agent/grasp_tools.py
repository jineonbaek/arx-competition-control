"""Read-only Agent tools composing recognition, pose and geometric preview."""
from pathlib import Path
from .contracts import AgentError,object_schema
from .tools import ToolRegistry,TEXT
from . import grasp_scene
from .grasp_pipeline import pipeline
from .battery_observation import observe

PREVIEW_TOOLS={"get_system_status","observe_scene","read_observation","capture_grasp_scene","preview_battery_grasp","observe_battery"}

class GraspTools(ToolRegistry):
    schemas={k:v for k,v in ToolRegistry.schemas.items() if k in PREVIEW_TOOLS}
    schemas.update(observe_battery=object_schema({"camera":{"type":"string","enum":["d435","d405"]}},["camera"]),
        capture_grasp_scene=object_schema(),
        preview_battery_grasp=object_schema({"observation_id":TEXT,
            "clearance_mm":{"type":"number","minimum":1,"maximum":200}},["observation_id","clearance_mm"]))
    descriptions={k:v for k,v in ToolRegistry.descriptions.items() if k in PREVIEW_TOOLS}
    descriptions.update(observe_battery="Freeze selected RGB-D camera, recognize one shape-defined battery and estimate its visible surface in base_link. D435 is coarse CAD localization; D405 is wrist-calibrated. Surface point is not a grip point. Returns explicit not-found/rejection and evidence ID. No robot motion.",
        capture_grasp_scene="Freeze paired D405 RGB-D and snapshot-time base_link transform using the existing owner. Temporary depth lease; no robot motion.",
        preview_battery_grasp="On one frozen D405 scene: reviewed battery recognition, FoundationPose, cloud evidence and +X constant-YZ contact-point path. TCP pending; no wrist targets or commands.")
    def __init__(self,config,store,*args,**kwargs):
        if config.motion_enabled or config.services:raise ValueError("Preview tools cannot have physical backends")
        super().__init__(config,store,*args,**kwargs)
        self.progress=lambda stage:None
    def _dispatch(self,name,a,call_id):
        try:
            if name=="observe_battery":return observe(self,a["camera"],call_id)
            if name=="capture_grasp_scene":return grasp_scene.capture(self)
            if name=="preview_battery_grasp":
                result=pipeline(self.store,a["observation_id"],a["clearance_mm"],
                    self.store.path/"previews"/call_id,self.progress)
                self.store.state["grasp_preview"]=str(Path("previews")/call_id/"result.json")
                return {"observation_id":a["observation_id"],"status":result["status"],"message":result["message"],
                    "tcp_status":"pending","executable":False,"result_file":self.store.state["grasp_preview"],
                    "depth":result.get("geometry",{}).get("depth"),"path":result.get("geometry",{}).get("path")}
            return super()._dispatch(name,a,call_id)
        except AgentError:raise
        except (OSError,ValueError,KeyError) as e:
            raise AgentError("preview_failed","预览证据或环境无效："+str(e)[:180]) from e
