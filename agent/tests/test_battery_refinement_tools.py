"""Fresh-scene refinement through the same Agent dispatch; synthetic evidence only."""
import copy
import json
import time
import unittest
from unittest.mock import patch
import numpy as np
from agent.battery_tools import BatteryExecutionTools
from agent.config import Config
from agent.mission import Mission
from agent.contracts import AgentError
from agent.storage import atomic_json
from agent.tests import test_battery_pick_plan as fixtures

POLICY=dict(max_corrections=2,max_target_shift_mm=[8.,8.,8.],max_step_mm=[6.,6.,6.],
    max_total_correction_mm=10.,position_tolerance_mm=[.5,.5,.5],
    orientation_tolerance_deg=.3,lift_tolerance_rad=.02)

class RefinementToolsTests(unittest.TestCase):
    preview=fixtures.BatteryPickPlanTests.preview
    plan=fixtures.BatteryPickPlanTests.plan
    def setUp(self):
        fixtures.BatteryPickPlanTests.setUp(self)
        self.tools=BatteryExecutionTools(Config(),self.store,observations=object(),
                                        pick_setup=self.setup,refinement_policy=POLICY)
    def arrived(self):
        self.preview();result=self.plan()
        self.assertTrue(result['ok'],result)
        previous=copy.deepcopy(result['data'])
        previous.update(observation_id='pre_motion',snapshot_id='pre_motion')
        self.store.state['mission'].update(stage='PICK_APPROACH_PLANNED',plan=previous)
        Mission(self.store.state['mission']).apply('execute_battery_approach',
            dict(pose_reached=True,new_observation_required=True),dict(plan=previous))
        self.store.state['mission'].setdefault('evidence_barriers',{})['PICK_AT_PREGRASP']=dict(
            completed_at=time.time()-1,previous_observation_ids=[])
    def test_initial_plan_is_staged_but_not_executable(self):
        self.preview();r=self.plan();self.assertTrue(r['ok'],r)
        self.assertEqual(self.store.state['mission']['stage'],'PICK_APPROACH_PLANNED')
        self.assertFalse(r['data']['executable'])
        self.assertTrue(r['data']['plan_id'])
        np.testing.assert_allclose(r['data']['actual_base_from_tool'],
            self.model.wrist_transform(self.q,self.lift)@np.array(self.setup['T_wrist_from_tool']))
        self.assertFalse(self.tools.execute('execute_battery_approach',
            {'plan_id':r['data']['plan_id']},'blocked')['ok'])
        self.assertIsNone(self.store.state['pending'])
    def test_new_reviewed_scene_can_align_without_moving(self):
        self.arrived();self.preview(call='fresh_preview');r=self.plan('fresh_plan')
        self.assertTrue(r['ok'],r)
        self.assertEqual(r['data']['refinement']['status'],'aligned')
        self.assertEqual(self.store.state['mission']['stage'],'PICK_PLANNED')
        self.assertFalse(r['data']['executable'])
    def test_changed_target_is_bounded_and_budget_counts_only_completed_correction(self):
        self.arrived()
        original=self.store.state['mission']['approach_plan']
        report=copy.deepcopy(self.report)
        for name in ('pre_mm','end_mm'):report['geometry']['path'][name][1]+=3.
        self.preview(report,'shifted');r=self.plan('correction')
        self.assertTrue(r['ok'],r)
        self.assertEqual(r['data']['refinement']['status'],'correction_required')
        self.assertEqual(self.store.state['mission'].get('pick_corrections',0),0)
        self.assertEqual(self.store.state['mission']['stage'],'PICK_CORRECTION_PLANNED')
        Mission(self.store.state['mission']).apply('execute_battery_approach',
            dict(pose_reached=True,new_observation_required=True),dict(plan=r['data']))
        self.assertEqual(self.store.state['mission']['pick_corrections'],1)
        self.assertAlmostEqual(self.store.state['mission']['pick_correction_mm'],3.,places=4)
        self.assertNotEqual(self.store.state['mission']['approach_plan'],original)
    def test_old_frame_failure_and_missing_policy_do_not_allow_insertion(self):
        self.arrived()
        self.store.state['mission']['evidence_barriers']['PICK_AT_PREGRASP']['previous_observation_ids']=[self.ident]
        r=self.plan('old');self.assertFalse(r['ok'])
        self.assertIsNone(self.store.state['mission']['plan'])
        self.store.state['mission']['evidence_barriers']['PICK_AT_PREGRASP']['previous_observation_ids']=[]
        self.tools.refinement_policy=None
        r=self.plan('no_policy');self.assertTrue(r['ok'],r)
        self.assertEqual(r['data']['refinement']['status'],'policy_pending')
        self.assertEqual(self.store.state['mission']['stage'],'PICK_AT_PREGRASP')
    def test_observation_budget_is_per_completed_motion_not_per_plan_retry(self):
        self.arrived();self.store.state['coarse_alignment']='fixture'
        obs=dict(status='localized',observation_id=self.ident)
        with patch('agent.grasp_tools.observe',return_value=obs):
            for call in ('one','two'):
                self.assertTrue(self.tools.execute('observe_battery',{'camera':'d405'},call)['ok'])
            self.assertEqual(self.tools.execute('observe_battery',{'camera':'d405'},'three')['error']['code'],'observation_budget')
            self.preview(call='again_preview');self.plan('again_plan')
            self.assertEqual(self.tools.execute('observe_battery',{'camera':'d405'},'four')['error']['code'],'observation_budget')
    def test_incomplete_arrival_never_changes_stage(self):
        self.store.state['mission']['stage']='PICK_APPROACH_PLANNED'
        with self.assertRaises(AgentError):
            Mission(self.store.state['mission']).apply('execute_battery_approach',{'pose_reached':True},{})
        self.assertEqual(self.store.state['mission']['stage'],'PICK_APPROACH_PLANNED')
    def test_dispatch_approach_then_new_scene_then_pick_never_claims_hold(self):
        import io
        import hashlib
        from agent.grasp_scene import save_frame,CALIBRATION
        calls=[]
        class Backend:
            def available(self,name):return name in ('execute_battery_approach','execute_pick')
            def invoke(self,name,args,request):
                calls.append(name)
                data=(dict(pose_reached=True,new_observation_required=True)
                      if name=='execute_battery_approach' else
                      dict(pose_reached=True,gripper_settled=True,grasp_confirmed=False))
                return dict(status='completed',execution_id=request,data=data)
        self.tools.backends=Backend();self.tools.config.motion_enabled=True
        def verified_fixture():
            result=self.store.state['mission']['plan']
            result.update(executable=True,ik_checked=True,collision_checked=True,
                          collision_free=True,environment_collision_checked=True)
            self.tools._save_pick_plan(result)
            return result['plan_id']
        self.preview();self.assertTrue(self.plan()['ok'])
        ident=verified_fixture()
        r=self.tools.execute('execute_battery_approach',{'plan_id':ident},'approach')
        self.assertTrue(r['ok'],r)
        self.assertEqual(self.store.state['mission']['stage'],'PICK_AT_PREGRASP')
        self.assertFalse(self.plan('old_scene')['ok'])
        f=copy.deepcopy(self.frame);f['id']=np.array('new_scene')
        for key in ('image_time','arm_time','body_time'):f[key]=np.array(time.time())
        buf=io.BytesIO();np.savez(buf,**f)
        obs=save_frame(self.store,buf.getvalue(),source='synthetic',
            calibration_sha256=hashlib.sha256(CALIBRATION.read_bytes()).hexdigest())
        self.ident=obs['observation_id']
        self.store.state['latest_battery_observations']['d405']=self.ident
        path=self.store.path/(self.ident+'.json')
        atomic_json(path,dict(status='localized',camera='d405',observation_id=self.ident))
        self.store.state['battery_observations'][self.ident]=path.name
        self.report['observation_id']=self.ident
        self.report['geometry']['snapshot_id']='new_scene'
        self.report['camera_handoff']['wrist_observation_id']=self.ident
        self.preview(call='new_preview');r=self.plan('new_plan');self.assertTrue(r['ok'],r)
        self.assertEqual(r['data']['refinement']['status'],'aligned')
        ident=verified_fixture()
        r=self.tools.execute('execute_pick',{'plan_id':ident},'insert')
        self.assertTrue(r['ok'],r)
        self.assertFalse(r['data']['grasp_confirmed'])
        self.assertEqual(self.store.state['mission']['stage'],'PICK_EXECUTED')
        self.tools.execute('execute_pick',{'plan_id':ident},'insert')
        self.assertFalse(self.tools.execute('execute_pick',{'plan_id':ident},'replay')['ok'])
        self.assertEqual(calls,['execute_battery_approach','execute_pick'])

    def test_modified_plan_or_changed_tcp_never_reaches_backend(self):
        self.preview();r=self.plan();ident=r['data']['plan_id']
        path=self.store.path/self.store.state['battery_pick_plan'];saved=path.read_bytes()
        path.write_text('{}')
        result=self.tools.execute('execute_battery_approach',{'plan_id':ident},'tampered')
        self.assertEqual(result['error']['code'],'plan_changed')
        path.write_bytes(saved)
        self.tools.pick_setup['calibration_id']='changed'
        result=self.tools.execute('execute_battery_approach',{'plan_id':ident},'changed_tcp')
        self.assertEqual(result['error']['code'],'plan_changed')
        self.assertIsNone(self.store.state['pending'])

    def test_model_cannot_supply_policy_or_permission(self):
        r=self.tools.execute('plan_battery_pick',{'observation_id':self.ident,'executable':True},'inject')
        self.assertFalse(r['ok']);self.assertIsNone(self.store.state['pending'])

if __name__=='__main__':unittest.main()
