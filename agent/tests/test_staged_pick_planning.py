import copy
import hashlib
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from agent.battery_tools import BatteryExecutionTools
from agent.battery_production import review_plan
from agent.config import Config
from agent.tests import test_battery_refinement_tools as refinement
from agent.tests import test_battery_pick_plan as fixtures
from web.src.handeye_column import ColumnModel


class StagedPickTests(unittest.TestCase):
    preview=fixtures.BatteryPickPlanTests.preview
    plan=fixtures.BatteryPickPlanTests.plan
    arrived=refinement.RefinementToolsTests.arrived
    def setUp(self):
        fixtures.BatteryPickPlanTests.setUp(self)
        self.store.state['battery_task_settings']={'vision_mode':'plan_f'}
        self.store.state['battery_flow']={'head':'head'}
        self.tools=BatteryExecutionTools(Config(),self.store,observations=object(),
            pick_setup=self.setup,refinement_policy=refinement.POLICY)

    def test_plan_f_direct_replan_keeps_insertion_scope(self):
        from agent.battery_pick_plan import plan
        self.preview()
        result=plan(self.tools,self.obs['observation_id'],'direct',self.setup)
        self.assertEqual(result.get('motion_scope'),'insertion')
        self.assertFalse(review_plan('execute_battery_approach',result,environment_clear=True)['ik_checked'])

    def test_deferred_geometry_does_not_publish_an_unwritten_receipt(self):
        from agent.battery_pick_plan import plan
        self.preview()
        result=plan(self.tools,self.obs['observation_id'],'deferred',self.setup,defer_motion=True)
        self.assertFalse((self.store.path/result['result_file']).exists())
        self.assertNotIn('battery_pick_plan',self.store.state)
        self.assertNotIn('battery_pick_plan_sha256',self.store.state)

    def test_cached_cad_rebuilds_after_source_changes(self):
        from agent.battery_pick_plan import planning_model
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'mesh';path.write_bytes(b'old')
            old=SimpleNamespace(source_hashes={str(path):hashlib.sha256(b'old').hexdigest()})
            registry=SimpleNamespace(_pick_model=old)
            self.assertIs(planning_model(registry),old)
            path.write_bytes(b'new')
            replacement=SimpleNamespace(source_hashes={str(path):hashlib.sha256(b'new').hexdigest()})
            with patch('agent.battery_pick_plan.ColumnModel',return_value=replacement) as build:
                self.assertIs(planning_model(registry),replacement)
                self.assertIs(planning_model(registry),replacement)
            self.assertEqual(build.call_count,1)

    def test_approach_only_plan_cannot_be_reviewed_for_insertion(self):
        self.preview();result=self.plan();self.assertTrue(result['ok'],result)
        p=result['data']
        self.assertEqual(p.get('motion_scope'),'approach')
        self.assertLessEqual(len(p['joint_waypoints_rad']),2)
        self.assertTrue(review_plan('execute_battery_approach',p,environment_clear=True)['ik_checked'])
        self.assertFalse(review_plan('execute_pick',p,environment_clear=True)['ik_checked'])

    def test_correction_does_not_compute_a_discarded_insertion_path(self):
        self.arrived()
        report=copy.deepcopy(self.report)
        for name in ('pre_mm','end_mm'):report['geometry']['path'][name][1]+=3.
        self.preview(report,'shift')
        import agent.battery_pick_plan as planner
        calls=[];original=planner.plan_insert
        def counted(*a,**kw):calls.append(kw.get('approach_only',False));return original(*a,**kw)
        with patch.object(planner,'plan_insert',side_effect=counted):result=self.plan('correction')
        self.assertTrue(result['ok'],result)
        self.assertEqual(result['data']['refinement']['status'],'correction_required')
        self.assertEqual(calls,[])
        self.assertEqual(result['data'].get('motion_scope'),'correction')
        self.assertFalse(review_plan('execute_pick',result['data'],environment_clear=True)['ik_checked'])

    def test_final_aligned_plan_is_insertion_and_cad_is_reused(self):
        self.arrived();self.preview(call='fresh')
        with patch('agent.battery_pick_plan.ColumnModel',wraps=ColumnModel) as build:
            result=self.plan('aligned')
        self.assertTrue(result['ok'],result)
        self.assertEqual(result['data'].get('motion_scope'),'insertion')
        self.assertEqual(build.call_count,0)
        self.assertTrue(review_plan('execute_pick',result['data'],environment_clear=True)['ik_checked'])


if __name__=='__main__':unittest.main()
