"""Preview-only UI on the existing Agent session and tool receipt framework."""
import json
from pathlib import Path
import uuid
from arxgent import ArxgentChat,ChatError
from agent.runtime import AgentRuntime
from agent.grasp_tools import GraspTools,PREVIEW_TOOLS
from agent.contracts import AgentError

class PreviewRuntime(AgentRuntime):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.tools.progress=lambda stage:self.progress({"event":stage}) if self.progress else None
        self.store.state.pop("grasp_preview",None)
        self.store.save()
    def _preview(self,text):
        try:distance=float(text.split()[1])
        except (IndexError,ValueError):raise AgentError("invalid_arguments","预抓距离无效")
        if not 1<=distance<=200:raise AgentError("invalid_arguments","预抓距离必须为 1–200 mm")
        self.store.state["messages"].append({"role":"user","content":text})
        def call(name,args):
            if self.progress:self.progress({"event":"tool","tool":name})
            result=self.tools.execute(name,args,uuid.uuid4().hex)
            if self.progress:self.progress({"event":"tool_result","tool":name,"ok":result["ok"]})
            if not result["ok"]:raise AgentError(result["error"]["code"],result["error"]["message"])
            return result["data"]
        try:
            obs=call("capture_grasp_scene",{})
            result=call("preview_battery_grasp",{"observation_id":obs["observation_id"],"clearance_mm":distance})
            answer=result["message"]
        except AgentError as error:
            if error.code=="cancelled":return self._outcome("cancelled","")
            answer="预览停止："+error.message
        self.store.state["messages"].append({"role":"assistant","content":answer})
        return self._outcome("model_finished",answer)
    def run(self,instruction=None):
        return self._preview(instruction) if instruction and instruction.startswith("/preview ") else super().run(instruction)
    def continue_turn(self,instruction):
        return self._preview(instruction) if instruction.startswith("/preview ") else super().continue_turn(instruction)

class GraspIntegration(ArxgentChat):
    registry_class=GraspTools
    allowed_tools=PREVIEW_TOOLS
    def __init__(self,config,root,**kwargs):
        kwargs.setdefault("runtime_factory",PreviewRuntime)
        super().__init__(config,root,**kwargs)
    def _available(self):
        # The one-click workflow needs no conversational model call; recognition owns its configured review client.
        return (False,"预览服务正在关闭") if self.closed else (True,None)
    def result(self,client):
        with self.lock:
            session=self._session(client)
            if not session["job"] or session["job"]["status"]!="done" or not session["run_dir"]:
                raise ChatError(409,"本次预览尚未完成")
            run=Path(session["run_dir"])
            try:
                state=json.loads((run/"state.json").read_text())
                relative=state.get("grasp_preview")
                if not relative:raise ChatError(404,"本次没有预览结果；请查看状态说明")
                path=(run/relative).resolve()
                if not path.is_relative_to(run.resolve()):raise ChatError(404,"结果路径无效")
                return json.loads(path.read_text())
            except (OSError,ValueError):raise ChatError(404,"本次预览结果不可用") from None
