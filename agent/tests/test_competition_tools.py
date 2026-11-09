import tempfile
import threading
import unittest
from pathlib import Path
from agent.competition_jobs import Jobs
from agent.competition_tools import BROWSER_TOOLS,CompetitionTools
from agent.config import Config
from agent.storage import RunStore

class ToolAcceptance(unittest.TestCase):
    def test_preview_and_finish_gate(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=RunStore.create(tmp)
            try:
                registry=CompetitionTools(Config(),store,allowed_tools=BROWSER_TOOLS)
                move=dict(arm="right",start_body4_m=[0,0,.3],end_body4_m=[.05,0,.3],rpy_rad=[0,0,0])
                data=registry.execute("move",move,"one")["data"]
                self.assertFalse(data["executed"])
                self.assertTrue(all(p[2]==.3 for p in data["waypoints"]))
                move["end_body4_m"][2]=.4
                self.assertFalse(registry.execute("move",move,"two")["ok"])
                self.assertFalse(registry.execute("finish",{},"three")["ok"])
                self.assertEqual({d["function"]["name"] for d in registry.definitions(available_only=True)},BROWSER_TOOLS)
            finally:
                store.close()

    def test_wait_cancel_and_scope(self):
        queue=Jobs()
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            gate=threading.Event()
            started=threading.Event()
            def first(_):
                started.set()
                gate.wait(3)
                return 42
            a=queue.submit(root,"first",first)["job_id"]
            self.assertTrue(started.wait(1))
            b=queue.submit(root,"second",lambda _:0)["job_id"]
            self.assertEqual(queue.cancel(root,b)["status"],"cancelled")
            self.assertEqual(queue.wait(root,[a],0)[0]["status"],"running")
            gate.set()
            self.assertEqual(queue.wait(root,[a,b],2)[0]["result"],42)
            queue.pool.shutdown()
