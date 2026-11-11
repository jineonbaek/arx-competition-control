"""Offline sequence tests; synthetic states, never hardware commands."""
import unittest
import numpy as np
from agent.motion_feedback import PickMonitor
from web.src.handeye_column import ColumnModel

class PickMonitorTests(unittest.TestCase):
    def setUp(self):
        self.model=ColumnModel()
        self.q=np.radians([0,20,40,10,0,-90])
        self.tcp=np.eye(4);self.tcp[:3,3]=[.12,.007,-.004]
        self.target=self.model.wrist_transform(self.q,12)@self.tcp
        self.policy=dict(tolerance=[1.,1.,1.,.5,.5,.5],max_speed=[1.]*6,
                         settle_s=.2,timeout_s=3.,stall_s=.8,max_age_s=.15)
        self.grip_policy=dict(tolerance=[.02],max_speed=[.2],settle_s=.2,
                              timeout_s=2.,stall_s=.8,max_age_s=.15)

    def make(self,tcp=True):
        return PickMonitor(self.model,self.tcp if tcp else None,self.target,
            expected_lift_rad=12.,lift_tolerance_rad=.02,max_pair_skew_s=.05,
            started_at=10.,arm_policy=self.policy,gripper_target=-1.2,gripper_policy=self.grip_policy)

    def state(self,t,grip=-3.):
        return dict(chassis_active=False,arms={'right':dict(joints=self.q.tolist(),
            received=t,fresh=True,gripper=grip)},body=dict(height=12.,received=t,fresh=True))

    def arrive(self,g):
        for t in (10.01,10.11,10.22):r=g.update(self.state(t),t,ready=True)
        self.assertEqual(r['status'],'ready_to_close')
        self.assertFalse(r['grasp_confirmed'])

    def test_rotating_at_constant_angular_error_is_not_stationary(self):
        from scipy.spatial.transform import Rotation
        g=self.make()
        g.stage='ready_to_close';g.arrival_sample=10.1;g.last_sample=10.1
        g.last_value=np.array([0,0,0,.4,0,0])
        actual=self.target.copy()
        actual[:3,:3]=self.target[:3,:3]@Rotation.from_rotvec(np.radians([0,.4,0])).as_matrix()
        from unittest.mock import patch
        with patch.object(self.model,'wrist_transform',return_value=actual@np.linalg.inv(self.tcp)):
            r=g.start_closing(self.state(10.2),10.2,ready=True)
        self.assertFalse(r['close_permitted']);self.assertTrue(r['stop_required'])

    def test_console_snapshot_and_gripper_protocol_with_fake_adapter(self):
        from web.tests.test_control import Controller,FakeAdapter,configured
        now=[10.]
        adapter=FakeAdapter();controller=Controller(configured(),adapter,lambda:now[0])
        g=self.make()
        def state(t,grip=-3.):
            now[0]=t
            controller.update_arm('right',self.q.tolist(),grip,[0.]*6)
            controller.update_body(12.,[0.]*4,0.,0.,0.)
            return controller.snapshot()
        for t in (10.01,10.11,10.22):g.update(state(t),t,ready=True)
        self.assertEqual(adapter.calls,[])
        r=g.start_closing(state(10.32),10.32,ready=True)
        self.assertTrue(r['close_permitted'])
        if r['close_permitted']:
            controller.submit({'type':'gripper','arm':'right','position':-1.2},'offline-test')
        self.assertEqual(len(adapter.calls),1)
        for t in (10.33,10.43,10.54):r=g.update(state(t,-1.2),t,ready=True)
        self.assertEqual(r['status'],'awaiting_hold_verification')
        self.assertFalse(r['grasp_confirmed'])
        self.assertEqual(len(adapter.calls),1)

    def test_tcp_pending_cannot_start(self):
        with self.assertRaisesRegex(ValueError,'TCP'):self.make(False)

    def test_full_sequence_requires_new_preclose_feedback_and_does_not_claim_hold(self):
        g=self.make();self.arrive(g)
        self.assertFalse(g.start_closing(self.state(10.22),10.23,ready=True)['close_permitted'])
        r=g.start_closing(self.state(10.32),10.32,ready=True)
        self.assertTrue(r['close_permitted']);self.assertEqual(r['status'],'closing')
        for t in (10.33,10.43,10.54):r=g.update(self.state(t,-1.2),t,ready=True)
        self.assertEqual(r['status'],'awaiting_hold_verification')
        self.assertFalse(r['grasp_confirmed']);self.assertFalse(r['close_permitted'])
        self.assertFalse(g.start_closing(self.state(10.6),10.6,ready=True)['close_permitted'])

    def test_underreach_blocks_closing(self):
        g=self.make();g.target[0,3]+=.013
        for t in (10.01,10.11,10.22,10.32,10.42,10.52,10.62,10.72,10.92):
            r=g.update(self.state(t),t,ready=True)
            self.assertFalse(r['close_permitted'])
        self.assertEqual(r['status'],'stalled')
        self.assertTrue(r['stop_required'])
        self.assertFalse(g.start_closing(self.state(11.),11.,ready=True)['close_permitted'])

    def test_close_requires_arrival_not_backend_receipt(self):
        g=self.make()
        self.assertFalse(g.start_closing(self.state(10.01),10.01,ready=True)['close_permitted'])
        self.assertEqual(g.stage,'inserting')

    def test_post_arrival_drift_or_stale_or_fault_revokes_close(self):
        for fault in ('drift','stale','ready','cancel','lift','base','clock'):
            with self.subTest(fault=fault):
                g=self.make();self.arrive(g);s=self.state(10.32)
                now=10.32
                if fault=='drift':s['arms']['right']['joints'][0]+=.05
                if fault=='stale':now=10.6
                if fault=='lift':s['body']['height']+=.1
                if fault=='base':s['chassis_active']=True
                if fault=='clock':now=10.1
                r=g.start_closing(s,now,ready=fault!='ready',cancelled=fault=='cancel')
                self.assertFalse(r['close_permitted']);self.assertTrue(r['stop_required'])
                self.assertFalse(g.start_closing(self.state(11),11,ready=True)['close_permitted'])

    def test_arm_drift_while_closing_stops_and_never_counts_as_success(self):
        g=self.make();self.arrive(g);g.start_closing(self.state(10.32),10.32,ready=True)
        s=self.state(10.42,-1.2);s['arms']['right']['joints'][1]+=.1
        r=g.update(s,10.42,ready=True)
        self.assertTrue(r['stop_required']);self.assertFalse(r['grasp_confirmed'])

    def test_gripper_command_or_one_sample_is_not_hold_or_arrival(self):
        g=self.make();self.arrive(g);g.start_closing(self.state(10.32),10.32,ready=True)
        self.assertEqual(g.update(self.state(10.42,-1.2),10.42,ready=True)['status'],'closing')
        self.assertEqual(g.update(self.state(10.42,-1.2),10.5,ready=True)['status'],'closing')
        self.assertEqual(g.update(self.state(10.42,-1.2),10.6,ready=True)['status'],'feedback_invalid')

if __name__=='__main__':unittest.main()
