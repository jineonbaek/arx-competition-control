"""Execution-policy parsing is pure software; no controller or transport."""
import copy
import json
import tempfile
import unittest
from pathlib import Path

from agent.execution_policy import load


def policy():
    arrival_arm=dict(tolerance=[2.,2.,2.,.5,.5,.5],max_speed=[2.]*6,
        settle_s=.25,timeout_s=8.,stall_s=2.,max_age_s=.3)
    arrival_scalar=dict(tolerance=[.02],max_speed=[.2],
        settle_s=.25,timeout_s=8.,stall_s=2.,max_age_s=.3)
    timing=dict(joint_speed_deg_s=[5.7]*6,joint_acceleration_deg_s2=[12.]*6,
        tcp_speed_mm_s=20.,tcp_acceleration_mm_s2=40.)
    return dict(
        clearance_mm=50.,
        alignment_tolerance_deg=5.,
        j6_tolerance_deg=5.,
        timing_limits=timing,
        timeout_s={'execute_coarse_alignment':60.,'execute_battery_approach':30.,'execute_pick':30.},
        coarse=dict(reach_bound_m=1.,fixed_lift_target_rad=5.02,fixed_joint_tolerance_rad=.02,
            max_pair_skew_s=.05,observer_search=dict(start_z_mm=700.5,rpy_deg=[-.3,-2.6,.1],
                offsets_mm=[10.,20.,30.,40.,0.,-10.,-20.,-30.],z_tolerance_mm=2.,
                orientation_tolerance_deg=5.),
            lift_policy=dict(arrival_scalar,timeout_s=30.),arm_policy=dict(arrival_arm,timeout_s=20.)),
        pick=dict(lift_tolerance_rad=.02,max_pair_skew_s=.05,
            arm_policy=arrival_arm,gripper_target=-.4,
            gripper_policy=arrival_scalar,hold_gripper_range=[-1.5,-.5]),
        refinement=dict(max_corrections=2,max_target_shift_mm=[15.]*3,
            max_step_mm=[15.]*3,max_total_correction_mm=30.,
            position_tolerance_mm=[18.]*3,orientation_tolerance_deg=5.,
            lift_tolerance_rad=.02))


class ExecutionPolicyTests(unittest.TestCase):
    def write(self,data):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        path=Path(temp.name)/'policy.json'
        path.write_text(json.dumps(data,allow_nan=True))
        return path

    def test_complete_policy_loads_without_side_effects(self):
        data=policy();result=load(self.write(data))
        self.assertEqual(result,data)
        result['pick']['arm_policy']['tolerance'][0]=999
        self.assertEqual(data['pick']['arm_policy']['tolerance'][0],2.)

    def test_unknown_missing_and_nonfinite_values_are_rejected(self):
        cases=[]
        value=policy();value['extra']=1;cases.append(value)
        value=policy();del value['pick'];cases.append(value)
        value=policy();value['timing_limits']['tcp_speed_mm_s']=float('nan');cases.append(value)
        value=policy();value['timeout_s']['execute_pick']=0;cases.append(value)
        value=policy();value['refinement']['max_corrections']=4;cases.append(value)
        for data in cases:
            with self.subTest(data=data),self.assertRaises(ValueError):
                load(self.write(data))

    def test_policy_shapes_and_controller_limits_are_checked(self):
        cases=[]
        value=policy();value['timing_limits']['joint_speed_deg_s']=[6.]*6;cases.append(value)
        value=policy();value['coarse']['lift_policy']['tolerance']=[.02,.02];cases.append(value)
        value=policy();value['pick']['arm_policy']['tolerance']=[1.]*3;cases.append(value)
        value=policy();value['pick']['gripper_target']=.1;cases.append(value)
        value=policy();value['pick']['hold_gripper_range']=[-1.6,-2.6];cases.append(value)
        value=policy();value['pick']['hold_gripper_range']=[-3.2,-.5];cases.append(value)
        value=policy();value['refinement']['position_tolerance_mm']=[1.,1.];cases.append(value)
        for data in cases:
            with self.subTest(data=data),self.assertRaises(ValueError):
                load(self.write(data),gripper_limits=(-3.14,0.))

    def test_invalid_json_and_permissions_fail_closed(self):
        path=self.write(policy());path.write_text('{"bad":NaN}')
        with self.assertRaises(ValueError):load(path)
        with self.assertRaises(OSError):load(path.parent/'missing.json')


if __name__=='__main__':unittest.main()
