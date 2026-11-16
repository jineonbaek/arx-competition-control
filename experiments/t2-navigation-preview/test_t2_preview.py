import threading,unittest
from t2_preview import T2PreviewModule

class Task:
    def __init__(self):self.calls=[]
    def start_owned(self,token):self.calls.append(token);return {"run_id":"real-run"}
class T2PreviewTests(unittest.TestCase):
    def context(self):return {"run_id":"competition-A","stage":"place","request_id":"competition-A:place","token":object(),"cancelled":threading.Event(),"controller":object(),"competition_token":object()}
    def test_preview_never_starts_live_worker_or_reports_success(self):
        task=Task();seen=[]
        m=T2PreviewModule(lambda ctx:seen.append(ctx["request_id"]) or {"executable":False,"insert":{"status":"kinematic_preview"}},task)
        ctx=self.context();self.assertEqual(m.start(ctx)["status"],"running");self.assertTrue(m.wait(ctx,1.))
        result=m.inspect(ctx)
        self.assertEqual(result["status"],"failed");self.assertIn("仅供预览",result["reason"])
        self.assertEqual(seen,[ctx["request_id"]]);self.assertEqual(task.calls,[])
    def test_duplicate_request_cannot_repeat_preview(self):
        seen=[];m=T2PreviewModule(lambda ctx:seen.append(1) or {"executable":False},Task());ctx=self.context();m.start(ctx);self.assertTrue(m.wait(ctx,1.))
        with self.assertRaises(ValueError):m.start(ctx)
        self.assertEqual(seen,[1])
    def test_mismatched_identity_never_completes(self):
        m=T2PreviewModule(lambda ctx:{"executable":False},Task());ctx=self.context();m.start(ctx)
        self.assertEqual(m.inspect(ctx|{"run_id":"other"})["status"],"unknown")
    def test_cancelled_before_start_does_not_invoke_planner(self):
        seen=[];m=T2PreviewModule(lambda ctx:seen.append(1),Task());ctx=self.context();ctx["cancelled"].set()
        self.assertEqual(m.start(ctx)["status"],"stopped");self.assertEqual(seen,[])
    def test_stop_does_not_claim_worker_exited_while_planner_is_blocked(self):
        started=threading.Event();release=threading.Event()
        def planner(ctx):
            started.set();release.wait(1.);return {"executable":False}
        m=T2PreviewModule(planner,Task(),stop_timeout=.01);ctx=self.context()
        try:
            m.start(ctx);self.assertTrue(started.wait(.2))
            self.assertFalse(m.stop(ctx)["stopped"])
        finally:release.set()
        self.assertTrue(m.wait(ctx,1.));self.assertTrue(m.stop(ctx)["stopped"])
        self.assertEqual(m.inspect(ctx)["status"],"stopped")
    def test_no_untrusted_executable_flag_enables_live_motion(self):
        task=Task();m=T2PreviewModule(lambda ctx:{"executable":True},task);ctx=self.context();m.start(ctx);self.assertTrue(m.wait(ctx,1.))
        self.assertEqual(m.inspect(ctx)["status"],"failed");self.assertEqual(task.calls,[])
if __name__=="__main__":unittest.main()
