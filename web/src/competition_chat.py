"""Isolated arxgent tool acceptance UI. Live state comes from the 9983 owner."""
import argparse
from http import cookies
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import json
from pathlib import Path
import re
from urllib.parse import urlsplit
import uuid

from arxgent import ArxgentChat, ChatError
from agent.competition_scene import MAIN
from agent.competition_tools import CompetitionTools, BROWSER_TOOLS
from agent.config import Config, load_private_env
from agent.runtime import AgentRuntime
from agent.contracts import AgentError,strict_json

ROOT=Path(__file__).resolve().parents[2]
PUBLIC=ROOT/"web/public"

class AcceptanceRuntime(AgentRuntime):
    def _manual(self,instruction):
        _,_,command=instruction.partition(" ")
        name,_,payload=command.strip().partition(" ")
        self.progress({"event":"tool","tool":name})
        result=self.tools.execute(name,payload.strip() or "{}",uuid.uuid4().hex)
        self.progress({"event":"tool_result","tool":name,"ok":result["ok"]})
        answer=json.dumps(result,ensure_ascii=False,indent=2)
        messages=self.store.state["messages"]
        if not messages:
            messages.append({"role":"system","content":self.system_context or ""})
        messages.extend([{"role":"user","content":instruction},{"role":"assistant","content":answer}])
        return self._outcome("model_finished",answer)

    def run(self,instruction=None):
        if instruction and instruction.startswith("/tool "):
            return self._manual(instruction)
        return super().run(instruction)

    def continue_turn(self,instruction):
        if instruction.startswith("/tool "):
            return self._manual(instruction)
        return super().continue_turn(instruction)

class AcceptanceChat(ArxgentChat):
    registry_class=CompetitionTools
    allowed_tools=BROWSER_TOOLS
    request_limit=96

class Handler(BaseHTTPRequestHandler):
    def client(self):
        try:
            jar=cookies.SimpleCookie(self.headers.get("Cookie",""))
            return str(uuid.UUID(jar["arx_acceptance"].value))
        except (KeyError,ValueError,cookies.CookieError):
            return None

    def reply(self,code,body,mime="application/json",cookie=None):
        if not isinstance(body,bytes):
            body=json.dumps(body,ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type",mime)
        self.send_header("Content-Length",str(len(body)))
        self.send_header("Cache-Control","no-store")
        self.send_header("X-Content-Type-Options","nosniff")
        self.send_header("Content-Security-Policy","default-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        if cookie:
            self.send_header("Set-Cookie",f"arx_acceptance={cookie}; HttpOnly; SameSite=Strict; Path=/")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path=urlsplit(self.path).path
        client=self.client()
        if path=="/":
            return self.reply(200,(PUBLIC/"competition.html").read_bytes(),"text/html; charset=utf-8",client or uuid.uuid4())
        if path in ("/css/style.css","/css/arxgent.css","/js/arxgent.js","/js/acceptance.js"):
            return self.reply(200,(PUBLIC/path[1:]).read_bytes(),"text/css" if path.endswith(".css") else "application/javascript")
        if not client:
            return self.reply(403,{"message":"请先打开验收页"})
        chat=self.server.chat
        if path=="/api/arxgent":
            return self.reply(200,chat.snapshot(client))
        if path=="/api/examples":
            return self.reply(200,EXAMPLES)
        match=re.fullmatch(r"/scene/(obs_[a-f0-9]{32})/(d405|d435|astra)\.jpg",path)
        if match:
            with chat.lock:
                run=chat._session(client)["run_dir"]
            if run:
                file=Path(run)/"observations"/match[1]/(match[2]+".jpg")
                if file.is_file():
                    return self.reply(200,file.read_bytes(),"image/jpeg")
            return self.reply(404,{"message":"本对话未保存该画面"})
        match=re.fullmatch(r"/evidence/([a-f0-9]{32})/(mask|overlay|candidates)\.png",path)
        if match:
            with chat.lock:
                run=chat._session(client)["run_dir"]
            if run:
                file=Path(run)/"jobs"/match[1]/(match[2]+".png")
                if file.is_file():
                    return self.reply(200,file.read_bytes(),"image/png")
        return self.reply(404,{"message":"未找到结果"})

    def do_POST(self):
        client=self.client()
        if (self.path!="/api/arxgent" or not client or
            self.headers.get("Origin")!="http://"+self.headers.get("Host","")):
            return self.reply(403,{"message":"需要同源验收会话"})
        try:
            length=int(self.headers.get("Content-Length","0"))
            if not 0<length<=8192 or self.headers.get_content_type()!="application/json":
                raise ValueError()
            state=self.server.chat.handle(client,strict_json(self.rfile.read(length)))
            return self.reply(202,state)
        except ChatError as e:
            return self.reply(e.status,{"message":e.message})
        except (AgentError,ValueError,TypeError):
            return self.reply(400,{"message":"请求格式有误"})

EXAMPLES={
"capture_scene":{"camera":"d405","rgbd":True},"view_env_state":{},
"locate · segment":{"method":"segment","observation_id":"OBS","camera":"d405"},
"locate · pixels":{"method":"pixels","observation_id":"OBS","points":[[240,320]]},
"locate · triangulate":{"method":"triangulate","observations":["OBS1","OBS2"],"points":[[240,320],[240,300]],"static_correspondence":True},
"generate_grasp_candidates":{"observation_id":"OBS","segment_job":"JOB"},
"get_tool_result":{"job_id":"JOB"},"queue_status":{},"list_tool_jobs":{},
"wait_for_tool":{"job_id":"JOB","timeout_s":5},"wait_for_tools":{"job_ids":["JOB"],"timeout_s":5},
"cancel_tool":{"job_id":"JOB"},"flush_queue":{},"list_skills":{},"read_skill":{"name":"fixed_z_insertion"},
"publish_insight":{"observation_id":"OBS","judgment":"uncertain","reason":"把手有遮挡"},
"move":{"arm":"right","start_body4_m":[0.2,0.1,0.3],"end_body4_m":[0.25,0.1,0.3],"rpy_rad":[0,0,0]},
"grasp":{"arm":"right"},"release":{"arm":"right"},"hold":{"arm":"right","seconds":1},"finish":{}
}

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--port",type=int,default=9986)
    args=parser.parse_args()
    load_private_env(MAIN/"agent/.secrets/qwen.env")
    server=ThreadingHTTPServer(("0.0.0.0",args.port),Handler)
    server.chat=AcceptanceChat(Config(),ROOT/"agent/runs",runtime_factory=AcceptanceRuntime)
    try:
        server.serve_forever()
    finally:
        server.chat.close()
        server.server_close()
