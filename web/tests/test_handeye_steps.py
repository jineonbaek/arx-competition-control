import sys, unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from handeye_steps import Steps

class Fake:
    def __init__(self):self.q=[0.]*6;self.sent=[]
    def snapshot(self):
        return {"ros":{"connected":True},"arms":{"right":{"joints":self.q,"fresh":True,"age_s":0,"timestamp":__import__("time").time()}},
                "body":{"fresh":True,"age_s":0,"height":0},"chassis_active":False,
                "limits":{"arms":{"right":{"joint_limits_deg":[[-89,89]]*6}}}}
    def submit(self,data,client):self.sent.append(data);return {"ok":True}

class WorkflowTests(unittest.TestCase):
    def setUp(self):self.c=Fake();self.w=Steps(self.c)
    def begin(self):return self.w.action("operator",dict(op="begin",dictionary="5X5_100",marker=1,mm=11.25))
    def test_only_owner_click_can_consume_one_ticket(self):
        s=self.begin();p=s["pending"];self.assertTrue(p["check"]["clear"]);self.assertFalse(self.c.sent)
        go=dict(op="execute",confirmed=True,session=s["session"],ticket=p["ticket"])
        with self.assertRaises(ValueError):self.w.action("other",go)
        with self.assertRaises(ValueError):self.w.action("operator",dict(go,confirmed=False))
        with patch("handeye_steps.threading.Thread.start"):
            self.w.action("operator",go)
        self.assertEqual(len(self.c.sent),1)
        with self.assertRaises(ValueError):self.w.action("operator",go)
        with self.assertRaises(ValueError):self.w.manual_submit({"type":"lift","target":2},"other")
    def test_changed_start_invalidates_confirmation(self):
        s=self.begin();self.c.q[1]=.2
        with self.assertRaises(ValueError):self.w.action("operator",dict(op="execute",confirmed=True,session=s["session"],ticket=s["pending"]["ticket"]))
        self.assertFalse(self.c.sent)
    def test_column_and_site_limit_skip_without_motion(self):
        self.c.q[0]=np.radians(-36)
        s=self.begin();self.assertIsNone(s["pending"]);self.assertEqual(len(s["history"]),13);self.assertFalse(self.c.sent)
