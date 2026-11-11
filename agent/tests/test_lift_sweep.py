import unittest
import numpy as np
from agent.lift_sweep import assess_lift, column_sweep
from web.src.handeye_column import ColumnModel,NOMINAL_RAD_PER_M

class LiftTests(unittest.TestCase):
    def frame(self):
        return dict(joints=np.zeros(6),left_joints=np.zeros(6),image_time=10.,arm_time=10.,
                    left_arm_time=10.,body_time=10.,lift_rad=2.)

    def test_both_real_arms_column_only_not_motion_permission(self):
        r=assess_lift(self.frame(),10.)
        self.assertEqual(r['status'],'column_clear')
        self.assertEqual(set(r['arms']),{'left','right'})
        self.assertFalse(r['environment_checked']);self.assertFalse(r['executable'])
        self.assertGreater(r['arms']['right']['samples'],2)

    def test_missing_stale_or_invalid_left_cannot_claim_both_arms_checked(self):
        for key,value in (('left_joints',None),('left_arm_time',9.),('left_joints',[0,0,float('nan'),0,0,0])):
            f=self.frame()
            if value is None:f.pop(key)
            else:f[key]=value
            result=assess_lift(f,10.)
            self.assertEqual(result['status'],'unavailable')
            self.assertFalse(result['both_arms_column_checked'])
        self.assertEqual(assess_lift(self.frame(),19.)['status'],'unavailable')

    def test_middle_collision_not_hidden_by_clear_endpoints(self):
        class Model:
            def pose(self,q,h):
                return ((abs(h/NOMINAL_RAD_PER_M-.1),'test_link'),None)
        r=column_sweep(Model(),np.zeros(6),0,.2*NOMINAL_RAD_PER_M)
        self.assertFalse(r['clear']);self.assertEqual(r['lower_mm'],0)

    def test_lowering_and_stationary_sweep_are_checked(self):
        model=ColumnModel()
        a=column_sweep(model,np.zeros(6),2,10)
        b=column_sweep(model,np.zeros(6),10,2)
        self.assertEqual(a['lower_mm'],b['lower_mm'])
        self.assertTrue(column_sweep(model,np.zeros(6),2,2)['clear'])

if __name__=='__main__':unittest.main()
