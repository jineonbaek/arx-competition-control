import unittest,threading
from unittest.mock import patch
import numpy as np
import fast_visual_level as f

class Model:
    reach=1.
    def wrist_transform(self,q,h): return np.eye(4)
class Adapter:
    def trajectory_status(self,side): return {"ready":True,"enabled":True,"driver_instance":"fake","command_epoch":1}
class Ctrl:
    def __init__(self):
        self.calls=[];self.adapter=Adapter();self.models={"right":Model(),"left":Model()}
        self.config={"arms":{"right":{"joint_limits_deg":[[-180,180]]*6}}}
        self.state={"arms":{"left":{"fresh":True,"joints":[0.]*6},"right":{"fresh":True,"joints":[0.]*6}},"body":{"fresh":True,"height":2.,"waist":0.,"head_yaw":0.,"head_pitch":0.},"chassis_active":False,"motion_reserved":False,"competition_reserved":False,"command_epoch":3}
    def snapshot(self): return self.state
    def reserve_motion(self,e): self.calls.append("reserve");return object()
    def reserve_chassis_maintenance(self): self.calls.append("maintenance");return object()
    def motion_valid(self,t): return True
    def release_chassis_maintenance(self,t): self.calls.append("release_maintenance")
    def release_motion(self,t): self.calls.append("release_motion")
class Timed:
    def __init__(self,q,*a,**k):self.q=np.asarray(q);self.times=np.array([0.,3.])
class Motion:
    def __init__(self,ctrl,token,**kwargs):self.ctrl=ctrl
    def arm(self,plans,guard,**kwargs):
        guard();self.ctrl.calls.append("arm");self.ctrl.state["arms"]["right"]["joints"]=plans[0]["joint_waypoints_rad"][-1]
    def lift(self,h,guard,**kwargs):
        guard();self.ctrl.calls.append("lift");self.ctrl.state["body"]["height"]=h
    def hold(self):self.ctrl.calls.append("hold")

class TestIntegration(unittest.TestCase):
    def test_construct_does_not_reserve_or_move(self):
        c=Ctrl();t=f.FastVisualLevel(c,object());self.assertEqual(c.calls,[]);self.assertEqual(t.snapshot()["status"],"idle")
    def test_clicked_execution_only_arm_then_lift_no_gripper_no_place(self):
        c=Ctrl();t=f.FastVisualLevel(c,object())
        with patch.object(f,"DockingMotion",Motion),patch.object(f,"TimedJointPath",Timed),patch.object(f,"empty_prepare_geometry",return_value=100.),patch.object(f,"motion_limits",return_value={}):
            t._run_empty()
        self.assertEqual(c.calls,["reserve","maintenance","arm","lift","release_maintenance","release_motion"])
        self.assertAlmostEqual(c.state["arms"]["right"]["joints"][5],-np.pi/2)
        self.assertEqual(c.state["body"]["height"],10.)
        self.assertEqual(t.snapshot()["status"],"prepared")
    def test_missing_feedback_fails_before_reserving(self):
        c=Ctrl();c.state["body"]["fresh"]=False;t=f.FastVisualLevel(c,object());t._run_empty()
        self.assertEqual(c.calls,[]);self.assertEqual(t.snapshot()["status"],"blocked")
    def test_failed_geometry_never_sends_motion_and_releases(self):
        c=Ctrl();t=f.FastVisualLevel(c,object())
        with patch.object(f,"DockingMotion",Motion),patch.object(f,"empty_prepare_geometry",side_effect=ValueError("clearance")):
            t._run_empty()
        self.assertNotIn("arm",c.calls);self.assertNotIn("lift",c.calls)
        self.assertEqual(c.calls[-2:],["release_maintenance","release_motion"])
        self.assertEqual(t.snapshot()["status"],"blocked")
    def test_arm_failure_holds_and_retains_control(self):
        class FailedMotion(Motion):
            def arm(self,*args,**kwargs):raise ValueError("arm failed")
        c=Ctrl();t=f.FastVisualLevel(c,object())
        with patch.object(f,"DockingMotion",FailedMotion),patch.object(f,"TimedJointPath",Timed),patch.object(f,"empty_prepare_geometry",return_value=100.),patch.object(f,"motion_limits",return_value={}):t._run_empty()
        self.assertIn("hold",c.calls);self.assertNotIn("lift",c.calls);self.assertNotIn("release_motion",c.calls)
        self.assertTrue(t.snapshot()["control_retained"])
    def test_at_target_does_not_issue_extra_commands(self):
        c=Ctrl();c.state["body"]["height"]=10.;c.state["arms"]["right"]["joints"][5]=-np.pi/2;t=f.FastVisualLevel(c,object())
        with patch.object(f,"DockingMotion",Motion),patch.object(f,"empty_prepare_geometry",return_value=100.):t._run_empty()
        self.assertNotIn("arm",c.calls);self.assertNotIn("lift",c.calls);self.assertEqual(t.snapshot()["status"],"prepared")
    def test_fixed_target_ignores_current_as_destination_but_checks_validity(self):
        q=np.radians([20.,60.,40.,25.,22.,-90.])
        result=f.empty_prepare_target(q)
        np.testing.assert_allclose(result,np.radians([0.,0.,0.,0.,0.,-90.]))
    def test_fixed_target_moves_other_joints_even_if_j6_is_ready(self):
        c=Ctrl();c.state["body"]["height"]=10.;c.state["arms"]["right"]["joints"]=[.3,0.,0.,0.,0.,-np.pi/2];t=f.FastVisualLevel(c,object())
        with patch.object(f,"DockingMotion",Motion),patch.object(f,"TimedJointPath",Timed),patch.object(f,"empty_prepare_geometry",return_value=100.),patch.object(f,"motion_limits",return_value={}):t._run_empty()
        self.assertIn("arm",c.calls);self.assertEqual(t.snapshot()["status"],"prepared")
    def test_lift_preflight_rechecks_if_robot_shifts(self):
        snapshots=[{"arms":{"right":{"joints":[0.]*6}},"body":{"height":1.}}, {"arms":{"right":{"joints":[.005]+[0.]*5}},"body":{"height":1.}}, {"arms":{"right":{"joints":[.005]+[0.]*5}},"body":{"height":1.}}, {"arms":{"right":{"joints":[.005]+[0.]*5}},"body":{"height":1.}}]
        calls=[]
        q,h=f.checked_lift_start(lambda:snapshots.pop(0),lambda q,h:calls.append(float(q[0])),lambda:None)
        self.assertEqual(len(calls),2);self.assertAlmostEqual(q[0],.005);self.assertEqual(h,1.)
    def test_lift_preflight_rejects_continuing_drift(self):
        counter=iter(range(6))
        def snapshot():return {"arms":{"right":{"joints":[next(counter)*.01]+[0.]*5}},"body":{"height":1.}}
        with self.assertRaisesRegex(ValueError,"起点持续变化"):f.checked_lift_start(snapshot,lambda q,h:None,lambda:None)
    def test_invalid_joint_target(self):
        for q in ([0]*5,[0]*5+[float("nan")]):
            with self.assertRaises(ValueError):f.empty_prepare_target(q)
if __name__=="__main__":unittest.main()
