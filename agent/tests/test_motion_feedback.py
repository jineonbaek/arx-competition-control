"""Synthetic feedback only; no controller, CAN or camera transport."""
import unittest
import numpy as np
from agent.motion_feedback import ArrivalMonitor, tool_feedback
from web.src.handeye_column import ColumnModel

class ArrivalTests(unittest.TestCase):
    def gate(self,target=(100.,0.,0.),tolerance=(1.,1.,1.)):
        return ArrivalMonitor(target,tolerance,started_at=10.,settle_s=.3,timeout_s=3.,
            stall_s=.8,max_age_s=.2,max_speed=(.5,)*len(target))

    def test_underreach_never_arrives_then_stall_latches(self):
        g=self.gate()
        for t in (10.1,10.2,10.5,10.8):
            r=g.update([87,0,0],t,t)
            self.assertFalse(r['arrived'])
        r=g.update([87,0,0],11.,11.)
        self.assertEqual(r['status'],'stalled');self.assertTrue(r['stop_required'])
        self.assertEqual(g.update([100,0,0],11.1,11.1)['status'],'stalled')

    def test_arrival_needs_new_post_command_samples_and_stationary_dwell(self):
        g=self.gate()
        self.assertFalse(g.update([100,0,0],10.,10.1)['arrived'])
        self.assertFalse(g.update([100,0,0],10.1,10.1)['arrived'])
        self.assertFalse(g.update([100,0,0],10.2,10.2)['arrived'])
        self.assertFalse(g.update([100,0,0],10.3,10.3)['arrived'])
        r=g.update([100,0,0],10.41,10.41)
        self.assertTrue(r['arrived']);self.assertFalse(r['stop_required'])
        self.assertEqual(r['sample_time'],10.41)

    def test_passing_through_target_is_not_settled(self):
        g=self.gate()
        for t,x in ((10.1,99.2),(10.2,99.8),(10.3,100.4),(10.4,100.9),(10.5,102)):
            self.assertFalse(g.update([x,0,0],t,t)['arrived'])

    def test_duplicate_feedback_cannot_look_like_new_samples(self):
        g=self.gate()
        g.update([100,0,0],10.1,10.1)
        self.assertFalse(g.update([100,0,0],10.1,10.2)['arrived'])
        r=g.update([100,0,0],10.1,10.4)
        self.assertEqual(r['status'],'feedback_lost')

    def test_cancel_fault_reordered_time_and_invalid_values_stop(self):
        for options,expected in (({'cancelled':True},'cancelled'),({'ready':False},'driver_fault')):
            r=self.gate().update([100,0,0],10.1,10.1,**options)
            self.assertEqual(r['status'],expected);self.assertTrue(r['stop_required'])
        for value,stamp,now in (([np.nan,0,0],10.1,10.1),([100,0],10.1,10.1),
                               ([100,0,0],10.3,10.1),([100,0,0],10.1,np.nan)):
            self.assertEqual(self.gate().update(value,stamp,now)['status'],'invalid_feedback')
        g=self.gate();g.update([99,0,0],10.2,10.2)
        self.assertEqual(g.update([99,0,0],10.1,10.3)['status'],'invalid_feedback')

    def test_timeout_and_scalar_lift_or_gripper(self):
        g=self.gate();self.assertEqual(g.update([99,0,0],13.1,13.1)['status'],'timeout')
        for target in (12.,-1.2):
            g=self.gate((target,),(.02,))
            for t in (10.1,10.2,10.3,10.41):r=g.update([target],t,t)
            self.assertTrue(r['arrived'])

    def test_gap_resets_dwell_and_a_terminal_result_cannot_resume(self):
        g=self.gate()
        g.update([100,0,0],10.1,10.1)
        g.update([100,0,0],10.2,10.2)
        self.assertFalse(g.update([100,0,0],10.6,10.6)['arrived'])
        g.update([100,0,0],10.7,10.7)
        self.assertFalse(g.update([100,0,0],10.8,10.8)['arrived'])
        self.assertTrue(g.update([100,0,0],10.91,10.91)['arrived'])
        g=self.gate();g.update([90,0,0],10.1,10.1,cancelled=True)
        self.assertEqual(g.update([100,0,0],10.5,10.5)['status'],'cancelled')

    def test_no_implicit_tolerances_or_nonfinite_configuration(self):
        for target,tol in (([1],[0]),([1],[np.nan]),([1,2],[1]),([np.nan],[1])):
            with self.assertRaises(ValueError):self.gate(target,tol)

class ToolFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.model=ColumnModel();self.q=np.radians([0,20,40,10,0,-90])
        self.tcp=np.eye(4);self.tcp[:3,3]=[.12,.007,-.004]
        self.expected=self.model.wrist_transform(self.q,12)@self.tcp
        self.state=dict(chassis_active=False,
            arms={'right':dict(joints=self.q.tolist(),received=10.1,fresh=True)},
            body=dict(height=12.,received=10.12,fresh=True))

    def test_actual_tool_position_comes_from_fk_and_explicit_tcp(self):
        r=tool_feedback(self.state,self.model,self.tcp,self.expected,now=10.15,
            max_age_s=.2,max_pair_skew_s=.05,expected_lift_rad=12.,lift_tolerance_rad=.02)
        np.testing.assert_allclose(r['position_error_mm'],[0,0,0],atol=1e-9)
        self.assertAlmostEqual(r['orientation_error_deg'],0)
        self.assertAlmostEqual(r['sample_time'],10.1)
        target=self.expected.copy();target[0,3]+=.013
        r=tool_feedback(self.state,self.model,self.tcp,target,now=10.15,
            max_age_s=.2,max_pair_skew_s=.05,expected_lift_rad=12.,lift_tolerance_rad=.02)
        self.assertAlmostEqual(r['position_error_mm'][0],-13)

    def test_missing_tcp_stale_pair_or_moving_base_lift_reject(self):
        import copy
        for change in ('tcp','fresh','pair','base','lift','age'):
            state=copy.deepcopy(self.state);tcp=self.tcp
            if change=='tcp':tcp=None
            if change=='fresh':state['arms']['right']['fresh']=False
            if change=='pair':state['body']['received']=10.3
            if change=='base':state['chassis_active']=True
            if change=='lift':state['body']['height']=12.1
            now=10.5 if change=='age' else 10.31 if change=='pair' else 10.15
            with self.assertRaises(ValueError):tool_feedback(state,self.model,tcp,self.expected,now=now,
                max_age_s=.2,max_pair_skew_s=.05,expected_lift_rad=12.,lift_tolerance_rad=.02)

if __name__=='__main__':unittest.main()
