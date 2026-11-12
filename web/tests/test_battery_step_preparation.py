"""Prepare the existing Controller steps; all feedback and transport are inert."""
import copy
import hashlib
import json
import unittest
import numpy as np
import test_pick_close_step as fixtures
from agent.battery_execution import prepare_step
from agent.pick_execution import ApproachTrajectoryStep, StationaryApproachStep, PickTrajectoryStep
from agent.grasp_path import TimedJointPath

class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.CloseStepTests();self.f.setUp();self.c=self.f.c
        self.c.wrist_reason=lambda *a:''
        self.c.config['arms']['right']['gripper_limits_raw']=[-3.14,0.]
        self.f.feedback(10.)
        self.timing=dict(joint_speed_deg_s=[5.7]*6,joint_acceleration_deg_s2=[12.]*6,
                         tcp_speed_mm_s=20.,tcp_acceleration_mm_s2=40.)
        self.setup=dict(calibration_id='synthetic-only',T_wrist_from_tool=self.f.tcp.tolist(),timing_limits=self.timing)
        self.policies=dict(pick=dict(lift_tolerance_rad=.02,max_pair_skew_s=.05,
            arm_policy=self.f.arm_policy,gripper_target=-1.2,
            gripper_policy=self.f.grip_policy,hold_gripper_range=[-2.6,-1.6]))
        q0=self.f.q.copy();q1=q0.copy();q1[0]+=.001
        a=(self.c.models['right'].wrist_transform(q0,12)@self.f.tcp)[:3,3]*1000
        b=(self.c.models['right'].wrist_transform(q1,12)@self.f.tcp)[:3,3]*1000
        self.plan=dict(arm='right',executable=True,ik_checked=True,collision_checked=True,
            collision_free=True,environment_collision_checked=True,calibration_id='synthetic-only',
            pick_setup_sha256=hashlib.sha256(json.dumps(self.setup,sort_keys=True,allow_nan=False).encode()).hexdigest(),
            T_wrist_from_tool=self.f.tcp.tolist(),R_base_from_tool=self.f.target[:3,:3].tolist(),
            start_joints_rad=q0.tolist(),lift_rad=12.,
            approach=dict(start_rad=q0.tolist(),end_rad=q0.tolist()),
            contact_path=dict(pre_mm=a.tolist(),end_mm=b.tolist()),joint_waypoints_rad=[q0.tolist(),q1.tolist()],
            refinement={'status':'aligned'})
        self.checks=[]
        def check(operation,plan,snapshot,now):
            self.checks.append(operation);return True
        self.preflight=check
    def prepare(self,operation='execute_battery_approach'):
        return prepare_step(operation,self.plan,self.c,self.f.token,policies=self.policies,
            pick_setup=self.setup,preflight=self.preflight)
    def test_stationary_approach_opens_then_waits_without_arm_trajectory(self):
        step=self.prepare();self.assertIsInstance(step.approach,StationaryApproachStep)
        step.start()
        for t in (10.01,10.11,10.22,10.32,10.42,10.53):
            self.f.feedback(t,grip=-3.14);r=step.poll()
        self.assertEqual(r['status'],'ready_to_reobserve')
        self.assertTrue(r['new_observation_required']);self.assertFalse(r['close_permitted'])
        self.assertEqual(len(self.f.adapter.calls),1)
        self.assertAlmostEqual(self.f.adapter.calls[0][1][2],-3.14)
        with self.assertRaises(ValueError):step.start()
    def test_moving_approach_and_insert_bind_real_steps_and_timing(self):
        # Move the synthetic start away from the fixed pregrasp pose.
        self.plan['start_joints_rad'][0]-=.001
        self.plan['approach']['start_rad']=self.plan['start_joints_rad'][:]
        step=self.prepare().approach;self.assertIsInstance(step,ApproachTrajectoryStep)
        self.assertEqual(set(step.plan),{'interpolation','time_from_start_s','joint_waypoints_rad'})
        np.testing.assert_allclose(step.monitor.target,self.f.target)
        pick=self.prepare('execute_pick');self.assertIsInstance(pick,PickTrajectoryStep)
        self.assertGreater(pick.plan['time_from_start_s'][-1],0)
        self.assertEqual(self.f.adapter.calls,[])
    def test_coarse_step_is_assembled_from_frozen_both_arm_feedback(self):
        import test_coarse_step as coarse
        f=coarse.CoarseStepTests();f.setUp()
        observer=f.q.copy();observer[0]+=.001
        policies=dict(coarse=dict(reach_bound_m=1.,timing_limits=self.timing,
            fixed_joint_tolerance_rad=.002,lift_policy=f.lift_policy,arm_policy=f.arm_policy,max_pair_skew_s=.05))
        p=dict(arm='right',executable=True,ik_checked=True,collision_checked=True,collision_free=True,
            environment_collision_checked=True,
            initial_joints_rad={'left':f.left,'right':f.q},
            lift={'fixed':True,'target_rad':6.,'current_rad':5.},
            observer={'status':'fixed_observer_candidate','joints_rad':observer,'column_path':{'clear':True}})
        f.step=prepare_step('execute_coarse_alignment',p,f.c,f.token,policies=policies,pick_setup=None,
                            preflight=lambda *a:True)
        self.assertEqual(f.adapter.calls,[])
        self.assertFalse(f.step.stationary)
        self.assertEqual(f.settle_lift()['status'],'observing_motion')
        self.assertEqual([c[0] for c in f.adapter.calls],['lift','trajectory'])

    def test_unverified_or_changed_tcp_rejected_before_transport(self):
        original=copy.deepcopy(self.plan)
        for key in ('executable','environment_collision_checked','collision_free'):
            self.plan=copy.deepcopy(original);self.plan[key]=False
            with self.assertRaises(ValueError):self.prepare()
        self.plan=original;self.setup['calibration_id']='changed'
        with self.assertRaises(ValueError):self.prepare()
        self.setup=None
        with self.assertRaises(ValueError):self.prepare()
        self.assertEqual(self.f.adapter.calls,[])
    def test_preflight_is_mandatory_and_rechecked_while_stationary(self):
        self.preflight=None
        with self.assertRaises(ValueError):self.prepare()
        allowed=[True]
        self.preflight=lambda *a:allowed[0]
        step=self.prepare();step.start();allowed[0]=False
        self.f.feedback(10.1);r=step.poll()
        self.assertTrue(r['stop_required']);self.assertFalse(r['grasp_confirmed'])
    def test_wrong_approach_endpoint_or_unreviewed_insert_rejected(self):
        self.plan['approach']['end_rad'][0]+=.02
        with self.assertRaises(ValueError):self.prepare()
        self.plan['refinement']['status']='correction_required'
        with self.assertRaises(ValueError):self.prepare('execute_pick')
        self.assertEqual(self.f.adapter.calls,[])

if __name__=='__main__':unittest.main()
