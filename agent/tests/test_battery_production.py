"""Production assembly stays inert and converts existing evidence deterministically."""
import json
import tempfile
import unittest
from pathlib import Path

from agent.battery_production import preflight,review_plan,workflow_factory
from agent.storage import RunStore
from agent.tests.test_execution_policy import policy


class Controller:
    config={'arms':{'right':{'gripper_limits_raw':[-3.14,0.]}}}
    def motion_valid(self,token):return token=='owned'


class ProductionTests(unittest.TestCase):
    def test_review_uses_existing_geometry_and_one_clear_area_confirmation(self):
        coarse=dict(lift={'fixed':True,'target_rad':5.52,
                       'column_sweep':{'both_arms_column_checked':True,'status':'column_clear'}},
            observer={'status':'fixed_observer_candidate','joints_rad':[0.]*6,
                      'column_path':{'clear':True}})
        rejected=review_plan('execute_coarse_alignment',coarse,environment_clear=False)
        self.assertTrue(rejected['ik_checked']);self.assertTrue(rejected['collision_free'])
        self.assertFalse(rejected['environment_collision_checked'])
        accepted=review_plan('execute_coarse_alignment',coarse,environment_clear=True)
        self.assertTrue(all(accepted[k] for k in ('ik_checked','collision_checked',
            'collision_free','environment_collision_checked')))
        pick=dict(status='kinematic_preview',joint_waypoints_rad=[[0.]*6,[.01]*6],
            column_checked=True,interpolation_position_bound_mm=1.9,
            sampled_orientation_error_deg=.6,timing={'status':'timed_preview'})
        self.assertTrue(review_plan('execute_pick',pick,environment_clear=True)['collision_free'])

    def test_preflight_keeps_fresh_feedback_and_lift_settled_gate(self):
        plan={k:True for k in ('executable','ik_checked','collision_checked','collision_free',
                               'environment_collision_checked')}
        plan['execution_review']={'environment_clear':True}
        snapshot=dict(chassis_active=False,motion_interrupted=False,
            arms={'left':{'fresh':True},'right':{'fresh':True}},body={'fresh':True},
            capabilities={'lift':'','left:arm.joints':'','right:arm.joints':'',
                          'right:arm.wrist':'','right:gripper':''})
        self.assertTrue(preflight('execute_battery_approach',plan,snapshot,1.))
        snapshot['capabilities']['right:arm.wrist']='等待升降停稳'
        self.assertFalse(preflight('execute_battery_approach',plan,snapshot,1.))
        coarse=dict(executable=True,ik_checked=True,collision_checked=True,
            collision_free=True,environment_collision_checked=True,
            execution_review={'environment_clear':True},lift={'fixed':True},
            observer={'status':'fixed_observer_candidate'})
        self.assertTrue(preflight('execute_coarse_alignment',coarse,snapshot,1.))
        coarse['observer']['status']='fixed_observer_unavailable'
        self.assertFalse(preflight('execute_coarse_alignment',coarse,snapshot,1.))

    def test_factory_loads_policy_and_constructs_without_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'policy.json';path.write_text(json.dumps(policy()))
            store=RunStore.create(Path(directory)/'runs')
            try:
                factory=workflow_factory(path,console_url='http://127.0.0.1:9983',
                    gripper_limits=(-3.14,0.))
                flow=factory(store,Controller(),'owned',True)
                self.assertEqual(flow.flow['phase'],'head')
                self.assertEqual(list((store.path/'executions').iterdir()),[])
            finally:store.close()


if __name__=='__main__':unittest.main()
