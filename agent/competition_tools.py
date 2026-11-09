"""Competition tools exposed in the isolated arxgent acceptance console."""
import math
import time
from urllib.error import HTTPError
from . import competition_scene as scene
from .competition_jobs import JOBS
from .contracts import AgentError, object_schema
from .competition_inference import segment, grasp_candidates
from .tools import ToolRegistry, CAMERA, TEXT

def array(item, minimum, maximum):
    return {"type":"array","items":item,"minItems":minimum,"maxItems":maximum}

PIXEL=array({"type":"integer","minimum":0},2,2)
IDS=array(TEXT,1,4)
SECONDS={"type":"number","minimum":0,"maximum":15}
XYZ=array({"type":"number","minimum":-5,"maximum":5},3,3)
def choice(*names):
    return {"type":"string","enum":list(names)}
def spec(**fields):
    return object_schema(fields)

SPECS={
    "capture_scene":(object_schema({"camera":CAMERA,"rgbd":{"type":"boolean"}},["camera"]),"Freeze one fresh image. Request rgbd=true on D405 for paired depth/body4 geometry."),
    "view_env_state":(object_schema({"observation_id":TEXT},[]),"Read current robot feedback or a historical observation by id."),
    "locate":(object_schema({"method":choice("segment","pixels","triangulate"),"camera":CAMERA,"observation_id":TEXT,
        "observations":IDS,"points":array(PIXEL,1,4),"static_correspondence":{"type":"boolean"}},["method"]),
        "segment queues shape-based battery recognition. pixels takes [row,col]. triangulate requires the same static feature across calibrated views."),
    "generate_grasp_candidates":(spec(observation_id=TEXT,segment_job=TEXT),"Queue GraspNet on a reviewed D405 battery mask and the same frozen RGB-D."),
    "move":(spec(arm=choice("left","right"),start_body4_m=XYZ,end_body4_m=XYZ,rpy_rad=XYZ),"Preview fixed-Z horizontal insertion geometry in body4. Returns path and execution readiness."),
    "grasp":(spec(arm=choice("left","right")),"Inspect measured gripper feedback and prepare closure."),
    "release":(spec(arm=choice("left","right")),"Inspect measured gripper feedback and prepare opening."),
    "hold":(spec(arm=choice("left","right"),seconds={"type":"number","minimum":0.5,"maximum":5}),"Observe fresh joint feedback over a time interval for stability."),
    "finish":(spec(),"Check the mission's verified placement completion gate."),
    "get_tool_result":(spec(job_id=TEXT),"Read one job receipt in this conversation."),
    "wait_for_tool":(spec(job_id=TEXT,timeout_s=SECONDS),"Wait bounded seconds, leaving a running job active."),
    "wait_for_tools":(spec(job_ids=IDS,timeout_s=SECONDS),"Wait for several receipts with one shared timeout."),
    "queue_status":(spec(),"Inspect this conversation's GPU jobs."),
    "list_tool_jobs":(spec(),"List this conversation's task receipts."),
    "cancel_tool":(spec(job_id=TEXT),"Cancel a queued job. Running jobs keep their receipts."),
    "flush_queue":(spec(),"Cancel this conversation's waiting GPU jobs."),
    "list_skills":(spec(),"List the competition procedures."),
    "read_skill":(spec(name=TEXT),"Read one versioned competition procedure."),
    "publish_insight":(spec(observation_id=TEXT,judgment=choice("selected","rejected","uncertain"),reason=TEXT),"Save an interpretation linked to its observation; not a motion authorization."),
}
SKILLS={
    "battery_shape":"按三个实拍视角比对壳体、凸把手、长圆通孔。颜色和贴纸不能决定目标身份；遮挡时报告不确定。",
    "handle_depth":"选择横梁实体表面，孔中心可能给出背景深度。缺少深度或标定时重新观测。",
    "fixed_z_insertion":"对准后在已验证规划坐标系保持全程 Z 和朝向，监测进度与偏差，插入后复核仓位、释放和稳定。",
}
BROWSER_TOOLS={"observe_scene","read_observation","get_system_status",*SPECS}

class CompetitionTools(ToolRegistry):
    schemas={**ToolRegistry.schemas,**{k:v[0] for k,v in SPECS.items()}}
    descriptions={**ToolRegistry.descriptions,**{k:v[1] for k,v in SPECS.items()}}

    def capabilities(self):
        return {**super().capabilities(), "generate_grasp_candidates":True}

    def definitions(self,*,available_only=False):
        rows=super().definitions(available_only=False)
        return rows if not available_only else [r for r in rows if r["function"]["name"] in BROWSER_TOOLS]

    def _dispatch(self,name,a,call_id):
        try:
            return self._competition(name,a,call_id)
        except HTTPError as error:
            raise AgentError("source_unavailable",f"采集服务返回 HTTP {error.code}，请检查设备状态") from error
        except (OSError,ValueError,KeyError,ImportError) as error:
            raise AgentError("evidence_unavailable",f"观测或运行环境不可用 ({type(error).__name__})") from error

    def _competition(self,name,a,call_id):
        root=self.store.path
        if name=="capture_scene":
            return scene.capture(self,a["camera"],a.get("rgbd",False))
        if name=="view_env_state":
            if "observation_id" in a:
                return self.store.observation(a["observation_id"])
            return super()._dispatch("get_system_status",{},call_id)
        if name=="locate":
            method=a["method"]
            if method=="triangulate":
                if a.get("static_correspondence") is not True:
                    raise AgentError("static_feature_required","必须确认各点属于同一静止特征，且 body4 参考系未移动")
                return scene.triangulate(self.store,a["observations"],a["points"])
            ident=a["observation_id"]
            camera=a.get("camera","d405")
            self.store.image_path(ident,camera)
            if method=="pixels":
                if camera!="d405":
                    raise AgentError("geometry_unavailable","当前标定几何来自 D405")
                return scene.pixels(self.store,ident,a["points"])
            if camera not in ("d435","d405"):
                raise AgentError("camera_unavailable","识别仅支持 D435/D405")
            return JOBS.submit(root,"locate.segment",lambda out:segment(self.store,ident,camera,out))
        if name=="generate_grasp_candidates":
            ident=a["observation_id"]
            scene.scene(self.store,ident)
            receipt=JOBS.record(root,a["segment_job"])
            return JOBS.submit(root,name,lambda out:grasp_candidates(self.store,ident,receipt,out))
        if name=="get_tool_result":
            return JOBS.record(root,a["job_id"])
        if name in ("wait_for_tool","wait_for_tools"):
            ids=[a["job_id"]] if name=="wait_for_tool" else a["job_ids"]
            return {"jobs":JOBS.wait(root,ids,a["timeout_s"])}
        if name in ("queue_status","list_tool_jobs"):
            return {"resource":"gpu","jobs":[{k:v for k,v in job.items() if k not in ("result","epoch")} for job in JOBS.list(root)]}
        if name=="cancel_tool":
            return JOBS.cancel(root,a["job_id"])
        if name=="flush_queue":
            return {"jobs":[JOBS.cancel(root,r["job_id"]) for r in JOBS.list(root) if r["status"]=="queued"]}
        if name=="list_skills":
            return {"skills":list(SKILLS),"revision":1}
        if name=="read_skill":
            if a["name"] not in SKILLS:
                raise AgentError("skill_missing","没有此经验条目")
            return {"name":a["name"],"revision":1,"text":SKILLS[a["name"]]}
        if name=="publish_insight":
            self.store.observation(a["observation_id"])
            records=self.store.state.setdefault("insights",[])
            data=dict(a,index=len(records),recorded_at=time.time())
            records.append(data)
            return data
        if name=="finish":
            return super()._dispatch("finish_task",{},call_id)
        if name=="move":
            start,end=a["start_body4_m"],a["end_body4_m"]
            if abs(start[2]-end[2])>1e-8:
                raise AgentError("fixed_z_required","起终点须保持相同 body4 Z")
            steps=max(1,math.ceil(math.dist(start,end)/.01))
            return {"mode":"geometric_preview","arm":a["arm"],"frame":"body4","unit":"m",
                "waypoints":[[start[0]+(end[0]-start[0])*i/steps,start[1]+(end[1]-start[1])*i/steps,start[2]] for i in range(steps+1)],
                "constant_rpy_rad":a["rpy_rad"],"executed":False,"ik_checked":False,"stop_reason":"verified_planner_required"}
        if name in ("grasp","release","hold"):
            def feedback():
                state=self.observations.robot_state()
                arm=state.get("snapshot",{}).get("arms",{}).get(a["arm"],{})
                if not arm.get("fresh"):
                    raise AgentError("feedback_unavailable","所选机械臂反馈不新鲜")
                return arm
            first=feedback()
            if name!="hold":
                return {"requested":name,"arm":a["arm"],"measured":first,"executed":False,
                        "stop_reason":"verified_gripper_backend_required"}
            start=time.monotonic()
            samples=[first]
            while time.monotonic()-start<a["seconds"]:
                time.sleep(.15)
                row=feedback()
                if row.get("timestamp")!=samples[-1].get("timestamp"):
                    samples.append(row)
            if len(samples)<2 or any(not s.get("joints") for s in samples):
                raise AgentError("insufficient_feedback","没有足够的独立关节反馈")
            spans=[max(j)-min(j) for j in zip(*(s["joints"] for s in samples))]
            return {"mode":"hold_observation","elapsed_s":time.monotonic()-start,"samples":len(samples),
                    "joint_spans_sdk":spans,"command_sent":False,"object_retention_verified":False}
        return super()._dispatch(name,a,call_id)
