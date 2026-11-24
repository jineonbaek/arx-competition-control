"""Synthetic measured states; no device or motion transport."""
import unittest
import numpy as np
from agent.motion_feedback import ArrivalMonitor, PickMonitor
from agent.mission import Mission
from agent.contracts import AgentError
from agent.pick_backend import ApproachExecutionBackend

class Model:
    def wrist_transform(self,q,lift):
        out=np.eye(4);out[:3,3]=q[:3];out[2,3]+=lift/41.54
        return out

class RelaxedTests(unittest.TestCase):
    def make(self,relaxed=True):
        p=dict(tolerance=[1.]*6,max_speed=[2.]*6,settle_s=.1,timeout_s=2.,stall_s=.5,max_age_s=.3)
        target=Model().wrist_transform([0]*6,12)
        return PickMonitor(Model(),np.eye(4),target,expected_lift_rad=12,
            lift_tolerance_rad=.05,max_pair_skew_s=.1,started_at=10.,arm_policy=p,
            gripper_target=0,gripper_policy=dict(p,tolerance=[.1],max_speed=[.2]),
            hold_gripper_range=[-1.5,-.5],arrival_mode='reference_settled' if relaxed else 'strict')
    def state(self,t,x=.013,lift=12.1):
        return dict(chassis_active=False,arms={'right':dict(joints=[x,0,0,0,0,0],fresh=True,
            received=t,gripper=-3.14)},body=dict(height=lift,fresh=True,received=t))
    def test_offset_is_advisory_only_after_reference_and_real_stationary_samples(self):
        g=self.make()
        self.assertFalse(g.update(self.state(10.1),10.1,ready=True)['close_permitted'])
        self.assertEqual(g.stage,'inserting')
        # No pre-reference samples may create an arrival receipt.
        g=self.make();g.begin_settling(10.)
        for t in (10.01,10.07,10.13):r=g.update(self.state(t),t,ready=True)
        self.assertEqual(r['status'],'ready_to_close')
        self.assertTrue(r['reference_settled']);self.assertFalse(r['pose_reached'])
        self.assertAlmostEqual(r['position_error_mm'][0],13.)
        self.assertAlmostEqual(r['lift_offset_rad'],.1)
        self.assertTrue(g.start_closing(self.state(10.16),10.16,ready=True)['close_permitted'])
    def test_moving_feedback_still_blocks_arrival_without_extra_preclose_gate(self):
        g=self.make();g.begin_settling(10.)
        for t,x in ((10.01,.013),(10.07,.014),(10.13,.015)):
            r=g.update(self.state(t,x),t,ready=True)
            self.assertEqual(r['status'],'inserting')
        for t in (10.19,10.25,10.31):r=g.update(self.state(t,.015),t,ready=True)
        self.assertEqual(r['status'],'ready_to_close')
        r=g.start_closing(self.state(10.37,.025),10.37,ready=True)
        self.assertFalse(r['stop_required']);self.assertTrue(r['close_permitted'])
    def test_stale_feedback_cancel_and_moving_chassis_remain_terminal(self):
        for fault in ('stale','cancel','base','driver'):
            g=self.make();g.begin_settling(10.);s=self.state(10.1)
            if fault=='base':s['chassis_active']=True
            r=g.update(s,10.5 if fault=='stale' else 10.1,
                       ready=fault!='driver',cancelled=fault=='cancel')
            self.assertTrue(r['stop_required'],fault)
    def test_legacy_strict_monitor_still_rejects_lift_offset(self):
        self.assertTrue(self.make(False).update(self.state(10.1),10.1,ready=True)['stop_required'])
    def test_no_target_precision_required_does_not_mean_constant_fake_zero_speed(self):
        g=ArrivalMonitor([0],[1],started_at=10.,settle_s=.1,timeout_s=2.,stall_s=.3,
                         max_age_s=.3,max_speed=[1],position_required=False)
        for t,x in ((10.1,10),(10.2,20),(10.4,30),(10.6,40)):
            self.assertEqual(g.update([x],t,t)['status'],'waiting')
        g.update([40],10.7,10.7)
        self.assertTrue(g.update([40],10.81,10.81)['arrived'])
    def test_backend_and_mission_do_not_fabricate_pose_reached(self):
        evidence=dict(status='ready_to_reobserve',close_permitted=False,new_observation_required=True,
                      arrival_mode='reference_settled',reference_settled=True,pose_reached=False,
                      position_error_mm=[13,0,0],return_trace_rad=[[0.]*6])
        data=ApproachExecutionBackend._completed_data(evidence)
        self.assertFalse(data['pose_reached']);self.assertTrue(data['reference_settled'])
        for enabled in (False,True):
            plan=dict(plan_id='p',arm='right')
            if enabled:plan['arrival_mode']='reference_settled'
            mission=Mission(dict(stage='PICK_APPROACH_PLANNED'))
            if enabled:mission.apply('execute_battery_approach',data,{'plan':plan})
            else:
                with self.assertRaises(AgentError):mission.apply('execute_battery_approach',data,{'plan':plan})

class TraceTests(unittest.TestCase):
    def test_approach_records_measured_start_and_end_without_breaking_base_step(self):
        from agent.pick_execution import ApproachTrajectoryStep,PickCloseStep
        from types import SimpleNamespace
        fixture=RelaxedTests();g=fixture.make();q=[.013,0,0,0,0,0]
        target=g.model.wrist_transform(q,12.);g.target=target
        c=SimpleNamespace(motion_valid=lambda t:True)
        # Base executor construction must retain the original ownership fields.
        self.assertIs(PickCloseStep(c,'token',g).controller,c)
        step=ApproachTrajectoryStep(c,'token',g,{'joint_waypoints_rad':[[0.]*6,q]},preflight=lambda s,n:True)
        step.preflight(fixture.state(10.1,0.,12.),10.1)
        step.preflight(fixture.state(10.2,.006,12.),10.2)
        g.latest_measured={'joints_rad':q}
        r=step._on_arrived({'status':'ready_to_close'})
        self.assertEqual(r['return_trace_rad'][0],[0.]*6)
        self.assertEqual(r['return_trace_rad'][-1],q)
    def test_motion_on_poll_after_arrival_has_no_added_gate(self):
        fixture=RelaxedTests();g=fixture.make();g.begin_settling(10.)
        for t in (10.01,10.07,10.13):g.update(fixture.state(t),t,ready=True)
        r=g.update(fixture.state(10.19,.025),10.19,ready=True)
        self.assertFalse(r['stop_required'])

class RemovedGateTests(unittest.TestCase):
    def test_relaxed_open_and_trajectory_do_not_repeat_runtime_cad(self):
        from unittest.mock import patch
        import test_battery_step_preparation as fixture
        t=fixture.PreparationTests();t.setUp()
        t.plan['arrival_mode']='reference_settled'
        t.plan['start_joints_rad'][0]-=.001
        t.plan['approach']['start_rad']=t.plan['start_joints_rad'][:]
        with patch.object(t.c.models['right'],'check',side_effect=AssertionError('Runtime CAD repeated')) as cad:
            step=t.prepare()
            self.assertEqual(step.start()['status'],'opening_gripper')
            with patch.object(t.c,'start_trajectory') as dispatch:
                self.assertEqual(step.approach.start()['status'],'awaiting_trajectory')
                guard=dispatch.call_args.kwargs['preflight']
                self.assertTrue(guard(t.c.snapshot(),t.f.now))
            cad.assert_not_called()
        self.assertEqual(len(t.f.adapter.calls),1)
        self.assertAlmostEqual(t.f.adapter.calls[0][1][2],-3.14)

    def test_original_plan_verification_still_required(self):
        import test_battery_step_preparation as fixture
        t=fixture.PreparationTests();t.setUp();t.plan['arrival_mode']='reference_settled'
        t.plan['collision_free']=False
        with self.assertRaises(ValueError):t.prepare()
        self.assertEqual(t.f.adapter.calls,[])

    def test_new_preclose_motion_and_paired_stamp_checks_are_removed(self):
        for paired_stamp in (False,True):
            fixture=RelaxedTests();g=fixture.make();g.begin_settling(10.)
            for t in (10.01,10.07,10.13):g.update(fixture.state(t),t,ready=True)
            moved=fixture.state(10.19,.025)
            if paired_stamp:moved['body']['received']=10.13
            self.assertFalse(g.update(moved,10.19,ready=True)['stop_required'])
            self.assertTrue(g.start_closing(fixture.state(10.25,.035),10.25,ready=True)['close_permitted'])
