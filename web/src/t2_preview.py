"""Offline T2 competition-module contract exercise; never commands hardware."""
import copy,threading,uuid

class T2PreviewModule:
    """Preview deliberately ends as failed: preview is not a placement receipt."""
    recoveries={}
    def __init__(self,planner,task,*,stop_timeout=2.):
        self.planner=planner;self.task=task;self.runs={};self.lock=threading.RLock();self.stop_timeout=stop_timeout
    def _record(self,context):
        record=self.runs.get(context.get("request_id"))
        if not record or record["competition_run_id"]!=context.get("run_id") or record["token"] is not context.get("token"):return None
        return record
    def start(self,context):
        request=context.get("request_id");run=context.get("run_id")
        if context.get("stage")!="place" or not isinstance(request,str) or not request or not isinstance(run,str) or not run or context.get("token") is None:
            raise ValueError("Invalid competition place context")
        with self.lock:
            if request in self.runs:raise ValueError("该任务已启动，禁止重复运行")
            if context["cancelled"].is_set():return {"status":"stopped","reason":"操作员停止"}
            record={"competition_run_id":run,"token":context["token"],"preview_id":"preview-"+uuid.uuid4().hex,"status":"running","result":None,"reason":"","stop":threading.Event(),"done":threading.Event()}
            self.runs[request]=record
            worker=threading.Thread(target=self._run,args=(context,record),name="offline-t2-preview",daemon=True);record["worker"]=worker;worker.start()
        return {"status":"running"}
    def _run(self,context,record):
        try:
            result=self.planner(context)
            if not isinstance(result,dict):raise ValueError("离线规划返回格式无效")
            with self.lock:
                record["result"]=copy.deepcopy(result)
                if context["cancelled"].is_set() or record["stop"].is_set():record.update(status="stopped",reason="操作员停止")
                else:record.update(status="failed",reason="路径仅供预览，未真实放置，不能发送完成回执")
        except Exception as error:
            with self.lock:
                if record["stop"].is_set() or context["cancelled"].is_set():record.update(status="stopped",reason="操作员停止")
                else:record.update(status="failed",reason="离线规划失败："+str(error))
        finally:record["done"].set()
    def wait(self,context,timeout):
        with self.lock:record=self._record(context)
        return False if record is None else record["done"].wait(timeout)
    def inspect(self,context):
        with self.lock:
            record=self._record(context)
            if not record:return {"status":"unknown","reason":"没有本轮 T2 记录"}
            return {"status":record["status"],"reason":record["reason"],"run_id":record["preview_id"],"preview":copy.deepcopy(record["result"])}
    def stop(self,context):
        with self.lock:
            record=self._record(context)
            if not record:return {"stopped":False}
            record["stop"].set();worker=record["worker"]
        worker.join(self.stop_timeout)
        with self.lock:
            if not worker.is_alive():record.update(status="stopped",reason="操作员停止")
        return {"stopped":not worker.is_alive()}
    def resume(self,context):raise ValueError("不能自动重放放置动作")
