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

    def test_insertion_path_uses_scene_and_horizontal_slot_axis(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=RunStore.create(tmp)
            try:
                obs=store.save_observation({"cameras":{"d405":{}}},{"d405":b"unit test"},geometry_archive=b"fixture")
                reg=CompetitionTools(Config(),store,allowed_tools=BROWSER_TOOLS)
                args=dict(observation_id=obs["observation_id"],slot_id="red",arm="right",
                    start_body4_m=[.2,.1,.3],axis_body4_xy=[0,1],travel_m=.05,rpy_rad=[0,0,0])
                result=reg.execute("insert_battery",args,"one")["data"]
                self.assertTrue(all(p[0]==.2 and p[2]==.3 for p in result["waypoints"]))
                self.assertAlmostEqual(result["waypoints"][-1][1],.15)
                self.assertFalse(result["executed"])
                args["axis_body4_xy"]=[0,0]
                self.assertFalse(reg.execute("insert_battery",args,"two")["ok"])
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
