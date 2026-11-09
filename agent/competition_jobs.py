"""One local GPU queue; receipts and results belong to the requesting run."""
from concurrent.futures import ThreadPoolExecutor
import json
import threading
import time
import uuid
from .storage import atomic_json
from .contracts import AgentError

class Jobs:
    def __init__(self):
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="competition-gpu")
        self.lock = threading.RLock()
        self.epoch = uuid.uuid4().hex

    def record(self, root, ident):
        if len(ident)!=32 or any(c not in "0123456789abcdef" for c in ident):
            raise AgentError("job_missing","无效任务编号")
        path=root/"jobs"/ident/"receipt.json"
        if not path.is_file():
            raise AgentError("job_missing","当前对话无此任务")
        with self.lock:
            data=json.loads(path.read_text())
            if data["epoch"]!=self.epoch and data["status"] in ("running","queued"):
                data["status"]="interrupted"
            return data

    def submit(self, root, name, work):
        with self.lock:
            if len(self.list(root))>=96:
                raise AgentError("job_limit","本对话任务数已达到上限")
            ident=uuid.uuid4().hex
            path=root/"jobs"/ident
            path.mkdir(parents=True)
            data=dict(job_id=ident,resource="gpu",tool=name,status="queued",epoch=self.epoch)
            atomic_json(path/"receipt.json",data)
            self.pool.submit(self._run,root,ident,work)
            return dict(data)

    def _run(self,root,ident,work):
        path=root/"jobs"/ident
        with self.lock:
            data=self.record(root,ident)
            if data["status"]!="queued":
                return
            data["status"]="running"
            atomic_json(path/"receipt.json",data)
        try:
            data["result"]=work(path)
            data["status"]="completed"
        except Exception as error:
            data.update(status="failed",error=str(error)[:450] if isinstance(error,AgentError) else type(error).__name__)
        with self.lock:
            atomic_json(path/"receipt.json",data)

    def list(self,root):
        return [self.record(root,p.parent.name) for p in sorted((root/"jobs").glob("*/receipt.json"))]

    def cancel(self,root,ident):
        with self.lock:
            row=self.record(root,ident)
            if row["status"]=="queued":
                row["status"]="cancelled"
                atomic_json(root/"jobs"/ident/"receipt.json",row)
            return row

    def wait(self,root,ids,timeout):
        end=time.monotonic()+timeout
        while True:
            rows=[self.record(root,ident) for ident in ids]
            if all(row["status"] not in ("queued","running") for row in rows) or time.monotonic()>=end:
                return rows
            time.sleep(min(.1,max(0,end-time.monotonic())))
JOBS=Jobs()
