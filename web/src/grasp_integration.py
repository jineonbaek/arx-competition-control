"""Preview-only UI on the existing Agent session and tool receipt framework."""
import json
from pathlib import Path
from arxgent import ArxgentChat,ChatError
from agent.runtime import AgentRuntime
from agent.grasp_tools import WorkflowTools,WORKFLOW_TOOLS

class GraspRuntime(AgentRuntime):
    prompt_path=Path(__file__).resolve().parents[2]/'agent/prompts/grasp.md'
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.tools.progress=lambda stage:self.progress({'event':stage}) if self.progress else None

class GraspIntegration(ArxgentChat):
    registry_class=WorkflowTools
    allowed_tools=WORKFLOW_TOOLS
    def __init__(self,config,root,**kwargs):
        kwargs.setdefault('runtime_factory',GraspRuntime)
        super().__init__(config,root,**kwargs)
    def result(self,client):
        with self.lock:
            session=self._session(client)
            if not session["job"] or session["job"]["status"]!="done" or not session["run_dir"]:
                raise ChatError(409,"本次预览尚未完成")
            run=Path(session["run_dir"])
            try:
                state=json.loads((run/"state.json").read_text())
                def read(relative):
                    path=(run/relative).resolve()
                    if not path.is_relative_to(run.resolve()):raise ChatError(404,'结果路径无效')
                    return json.loads(path.read_text())
                result=read(state['grasp_preview']) if state.get('grasp_preview') else {
                    'status':'stopped_before_fine_preview','executable':False,'tcp_status':'pending'}
                result['message']=(state.get('outcome') or {}).get('answer') or result.get('message','本轮已结束')
                result['coarse_alignment']=read(state['coarse_alignment']) if state.get('coarse_alignment') else None
                result['observations']={}
                for camera,ident in state.get('latest_battery_observations',{}).items():
                    relative=state.get('battery_observations',{}).get(ident)
                    if relative:result['observations'][camera]=read(relative)
                if not result['observations'] and not state.get('grasp_preview'):
                    raise ChatError(404,'本轮尚未取得观察证据；请查看 Agent 状态说明')
                return result
            except (OSError,ValueError):raise ChatError(404,"本次预览结果不可用") from None
