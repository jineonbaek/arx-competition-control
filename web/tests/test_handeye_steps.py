import sys, unittest
from pathlib import Path
from unittest.mock import Mock,patch
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
    def setUp(self):self.c=Fake();self.depth=Mock();self.w=Steps(self.c,self.depth)
    def begin(self):return self.w.action("operator",dict(op="begin",dictionary="5X5_100",marker=1,mm=11.25))
    def test_only_owner_click_can_consume_one_ticket(self):
        s=self.begin();p=s["pending"];self.assertTrue(p["check"]["clear"]);self.assertFalse(self.c.sent);self.depth.acquire_handeye.assert_not_called()
        go=dict(op="execute",confirmed=True,session=s["session"],ticket=p["ticket"])
        with self.assertRaises(ValueError):self.w.action("other",go)
        with self.assertRaises(ValueError):self.w.action("operator",dict(go,confirmed=False))
        with patch("handeye_steps.threading.Thread.start"):
            self.w.action("operator",go)
        self.assertEqual(len(self.c.sent),1);self.depth.acquire_handeye.assert_called_once_with(False)
        with self.assertRaises(ValueError):self.w.action("operator",go)
        with self.assertRaises(ValueError):self.w.manual_submit({"type":"lift","target":2},"other")
    def test_probe_failure_releases_depth_without_motion(self):
        from threading import Event
        released=Event()
        self.depth.release_handeye.side_effect=lambda token:released.set()
        with patch.object(self.w,"feedback",side_effect=[(self.c.snapshot(),np.zeros(6),0),ValueError("offline")]):
            self.w.action("operator",dict(op="probe",dictionary="5X5_100",marker=1,mm=11.25))
            self.assertTrue(released.wait(2))
        self.depth.acquire_handeye.assert_called_once_with(False)
        self.depth.release_handeye.assert_called_once_with(self.depth.acquire_handeye.return_value)
        self.assertFalse(self.c.sent)
        self.assertFalse(self.w.state("operator")["active"])

    def test_raised_anchor_is_required_and_does_not_accumulate(self):
        s=self.begin()
        def advance():return self.w.action("operator",dict(op="next",session=s["session"]))
        first=s["pending"]
        np.testing.assert_allclose(np.degrees(first["target"]),[0,0,5,3,0,0])
        self.assertGreater(first["wrist_rise_mm"],30)
        self.assertEqual(advance()["pending"]["point"],1)
        self.c.q=first["target"]
        self.w.raised=True
        for i in range(2,s["total"]+1):
            candidate=advance()["pending"]
            self.assertEqual(candidate["point"],i)
            np.testing.assert_allclose(np.degrees(candidate["target"][:4]),[0,0,5,3])
            self.assertGreater(candidate["wrist_rise_mm"],0)
            self.c.q=candidate["target"]
        self.assertFalse(self.c.sent)

    def test_supplement_keeps_current_arm_height_and_uses_two_wrist_axes(self):
        self.c.q=np.radians([0,0,10,6,-2,8]).tolist()
        anchor=np.array(self.c.q)
        state=self.w.action("operator",dict(op="begin",profile="supplement",
            dictionary="5X5_100",marker=26,mm=11.25,layout="CC200-15-11.25"))
        offsets=[]
        for i in range(1,state["total"]+1):
            p=state["pending"]
            self.assertEqual(p["profile"],"supplement")
            delta=np.degrees(np.asarray(p["target"])-anchor)
            np.testing.assert_allclose(delta[:4],0)
            offsets.append(delta[4:])
            self.c.q=p["target"]
            state=self.w.action("operator",dict(op="next",session=state["session"]))
        self.assertIsNone(state["pending"])
        np.testing.assert_allclose(np.ptp(offsets,axis=0),[40,40])
        self.assertFalse(self.c.sent)

    def test_changed_start_invalidates_confirmation(self):
        s=self.begin();self.c.q[1]=.2
        with self.assertRaises(ValueError):self.w.action("operator",dict(op="execute",confirmed=True,session=s["session"],ticket=s["pending"]["ticket"]))
        self.assertFalse(self.c.sent)
    def test_column_and_site_limit_skip_without_motion(self):
        self.c.q[0]=np.radians(-36)
        s=self.begin();self.assertIsNone(s["pending"]);self.assertEqual(len(s["history"]),2);self.assertFalse(self.c.sent)
