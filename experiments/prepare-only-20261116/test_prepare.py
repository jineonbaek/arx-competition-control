import unittest
import numpy as np
from fast_visual_level import prepare_joint_target,run_prepare_only

class PrepareTest(unittest.TestCase):
    def test_only_j6_changes(self):
        q=np.radians([10,20,30,40,50,10]);goal=prepare_joint_target(q)
        np.testing.assert_allclose(goal[:5],q[:5]);self.assertAlmostEqual(goal[5],-np.pi/2)
        np.testing.assert_allclose(q,np.radians([10,20,30,40,50,10]))
    def test_invalid_joints_are_rejected(self):
        for q in ([0]*5,[0]*5+[float("nan")]):
            with self.assertRaises(ValueError):prepare_joint_target(q)
    def test_prepare_stops_after_lift_and_retains_clamp(self):
        seen=[]
        run_prepare_only([0.]*6,5.,lambda q:seen.append(("arm",list(q))),lambda h:seen.append(("lift",h)))
        self.assertEqual([x[0] for x in seen],["arm","lift"])
        self.assertEqual(seen[-1][1],10.)
    def test_arm_failure_does_not_issue_lift(self):
        seen=[]
        def fail(q): raise ValueError("arm fail")
        with self.assertRaises(ValueError):run_prepare_only([0.]*6,5.,fail,lambda h:seen.append(h))
        self.assertEqual(seen,[])
if __name__=="__main__":unittest.main()
