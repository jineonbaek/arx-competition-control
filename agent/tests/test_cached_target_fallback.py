"""Cached Plan F geometry with inert flow tools and the existing offline IK fixture."""
import copy
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from agent.battery_flow import BatteryFlow
from agent.battery_tools import BatteryExecutionTools
from agent.contracts import AgentError
from agent.tests import test_battery_flow as flow_fixture
from agent.tests import test_battery_pick_plan as ik_fixture
from agent.tests.test_execution_policy import policy
from agent.battery_execution import prepare_step
from agent.mission import Mission

review=flow_fixture.review


class CacheFlowTests(unittest.TestCase):
    setUp=flow_fixture.FlowTests.setUp

    def prepare(self,*,with_b=False,failures=0,error='status',cancel=False):
        self.tools.vision_mode='plan_f'
        self.store.state['battery_task_settings']={'vision_mode':'plan_f'}
        self.tools.corrections=int(with_b)
        base=self.tools.execute
        self.sources=[]
        def execute(name,args,ident):
            m=self.store.state['mission']
            # Fail the first pregrasp frame, or the frame after one successful correction.
            if name=='observe_battery' and m.get('approach_plan') and (
                    not with_b or self.tools.calls.count('execute_battery_approach')>=2) and error!='bump':
                if cancel:(self.store.path/'cancel.flag').touch()
                if error=='exception':raise ValueError('OpenCV line failed')
                if error=='driver':raise AgentError('execution_unknown','driver receipt unknown')
                if error=='post_action':raise AgentError('post_action_evidence_required','new frame predates action')
                return dict(ok=True,data={'status':'depth_rejected','message':'depth insufficient'})
            if name=='plan_battery_pick' and m.get('approach_plan') and error=='bump':
                raise AgentError('pick_plan_failed','电池凸起朝向与约定夹姿不兼容')
            result=base(name,args,ident)
            if name=='plan_battery_pick' and result['ok']:
                result['data'].update(status='kinematic_preview',observation_id=args['observation_id'])
                self.store.state['grasp_preview']=ident+'.json'
            return result
        def cached(source,ident):
            self.sources.append(source)
            if len(self.sources)<=failures:raise ValueError('IK rejected')
            p=dict(plan_id=ident,arm='right',executable=False,cached_target={'source':source})
            self.store.state['mission'].update(stage='PICK_PLANNED',plan=p)
            return p
        self.tools.execute=execute
        self.tools.replan_cached_target=cached
        return BatteryFlow(self.tools,review_plan=review,vision_mode='plan_f')

    def test_first_pregrasp_failure_uses_a_and_finishes_original_tail_once(self):
        flow=self.prepare();result=flow.run()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(self.sources,['A'])
        self.assertEqual(set(flow.flow['target_cache']),{'A'})
        self.assertEqual(self.tools.calls[-4:],['execute_pick','verify_hold','stow_for_carry','return_to_carry'])

    def test_b_first_then_a_and_three_total_planning_attempts(self):
        flow=self.prepare(with_b=True,failures=2);result=flow.run()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(self.sources,['B','A','A'])
        self.assertEqual(flow.flow['fallback_attempts'],3)
        self.assertEqual(set(flow.flow['target_cache']),{'A','B'})
        self.assertNotEqual(flow.flow['target_cache']['A']['plan']['observation_id'],
                            flow.flow['target_cache']['B']['plan']['observation_id'])

    def test_three_failures_exit_and_never_send_pick(self):
        flow=self.prepare(failures=3);result=flow.run()
        self.assertEqual(result['status'],'needs_attention')
        self.assertEqual(self.sources,['A','A','A'])
        self.assertIn('IK rejected',result['reason'])
        self.assertIn('depth insufficient',result['reason'])
        self.assertNotIn('execute_pick',self.tools.calls)
        before=list(self.sources);flow.run();self.assertEqual(before,self.sources)

    def test_drawing_exception_after_approach_uses_cache(self):
        self.assertEqual(self.prepare(error='exception').run()['status'],'completed')
        self.assertEqual(self.sources,['A'])

    def test_bump_angle_rejection_at_b_uses_a_without_finishing_run(self):
        flow=self.prepare(error='bump');result=flow.run()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(self.sources,['A'])
        self.assertNotIn('B',flow.flow['target_cache'])
        self.assertIn('电池凸起朝向',flow.flow['fallback_reason'])

    def test_unusable_new_post_action_frame_uses_bound_old_target(self):
        self.assertEqual(self.prepare(error='post_action').run()['status'],'completed')
        self.assertEqual(self.sources,['A'])

    def test_cancel_or_unknown_motion_does_not_enter_fallback(self):
        result=self.prepare(error='driver').run()
        self.assertEqual(result['status'],'needs_attention')
        self.assertEqual(self.sources,[])

    def test_cancel_during_vision_does_not_enter_fallback(self):
        result=self.prepare(cancel=True).run()
        self.assertEqual(result['status'],'cancelled')
        self.assertEqual(self.sources,[])

    def test_motion_failure_after_cached_plan_never_tries_next_source(self):
        flow=self.prepare();self.tools.fail='execute_pick'
        self.assertEqual(flow.run()['status'],'needs_attention')
        self.assertEqual(self.sources,['A'])
        self.assertEqual(self.tools.calls.count('execute_pick'),1)

    def test_visual_error_without_cache_still_stops(self):
        flow=self.prepare();self.tools.fail='observe_battery'
        self.assertEqual(flow.run()['status'],'needs_attention')
        self.assertEqual(self.sources,[])

    def test_normal_plan_f_chain_never_enters_cache_retry(self):
        self.tools.vision_mode='plan_f'
        self.tools.replan_cached_target=lambda *a:self.fail('unexpected fallback')
        flow=BatteryFlow(self.tools,review_plan=review,vision_mode='plan_f')
        self.assertEqual(flow.run()['status'],'completed')
        self.assertNotIn('fallback_attempts',flow.flow)


class CachedIKTests(unittest.TestCase):
    setUp=ik_fixture.BatteryPickPlanTests.setUp
    preview=ik_fixture.BatteryPickPlanTests.preview
    plan=ik_fixture.BatteryPickPlanTests.plan

    def prepare(self):
        self.store.state['battery_task_settings']={'vision_mode':'plan_f'}
        self.store.state['battery_flow']={'head':self.ident,'target_cache':{}}
        self.current=self.q.copy();self.current[0]+=.001
        self.backend=SimpleNamespace(controller=SimpleNamespace(snapshot=lambda:dict(
            arms={'right':{'joints':self.current.tolist()}},body={'height':self.lift})))
        self.tools=BatteryExecutionTools(self.tools.config,self.store,observations=object(),
                                       backends=self.backend,pick_setup=self.setup)
        self.preview();initial=self.plan()['data']
        self.cache=dict(plan=copy.deepcopy(initial),preview_file=self.store.state['grasp_preview'])
        self.store.state['battery_flow']['target_cache']['A']=self.cache
        self.store.state['mission']['approach_plan']=copy.deepcopy(initial)
        self.store.state['latest_battery_observations']['d405']='new-failed-frame'
        self.store.state.pop('grasp_preview',None)
        return initial

    def test_cached_geometry_uses_current_feedback_and_keeps_original_binding(self):
        initial=self.prepare();saved=copy.deepcopy(self.cache)
        result=self.tools.replan_cached_target('A','cached_1')
        np.testing.assert_allclose(result['start_joints_rad'],self.current)
        self.assertNotEqual(initial['start_joints_rad'],result['start_joints_rad'])
        self.assertEqual(result['contact_path'],initial['contact_path'])
        self.assertEqual(result['R_base_from_tool'],initial['R_base_from_tool'])
        self.assertEqual(result['preview_sha256'],initial['preview_sha256'])
        self.assertEqual(self.cache,saved)
        self.assertEqual(result['vision_mode'],'d405')
        self.assertNotIn('refinement',result)
        self.assertNotIn('pickup_flow',result)
        self.assertEqual(self.store.state['mission']['stage'],'PICK_PLANNED')

    def test_changed_cached_evidence_or_setup_still_rejects(self):
        self.prepare();self.tools.pick_setup['calibration_id']='changed'
        with self.assertRaisesRegex(ValueError,'pick_setup_sha256'):
            self.tools.replan_cached_target('A','changed_setup')
        self.tools.pick_setup=copy.deepcopy(self.setup)
        (self.store.path/self.cache['preview_file']).write_text('{}')
        with self.assertRaises(ValueError):self.tools.replan_cached_target('A','changed_preview')

    def test_cached_execution_uses_original_checks_but_not_latest_failed_frame(self):
        self.prepare();result=self.tools.replan_cached_target('A','cached_1')
        result.update(review('execute_pick',result),executable=True)
        self.store.state['mission']['plan']=copy.deepcopy(result);self.tools._save_pick_plan(result)
        with patch('agent.tools.ToolRegistry._operation',return_value={'executed':True}) as operation:
            out=self.tools._operation('execute_pick',{'plan_id':result['plan_id']},'pick',
                                     Mission(self.store.state['mission']))
        self.assertTrue(out['executed']);operation.assert_called_once()
        result['ik_checked']=False
        self.store.state['mission']['plan']=result;self.tools._save_pick_plan(result)
        with self.assertRaises(AgentError):
            self.tools._operation('execute_pick',{'plan_id':result['plan_id']},'bad',
                                 Mission(self.store.state['mission']))

    def test_cached_plan_constructs_original_pick_step_without_fake_alignment(self):
        self.prepare();result=self.tools.replan_cached_target('A','cached_step')
        result.update(review('execute_pick',result),executable=True)
        settings=policy()
        controller=SimpleNamespace(models={'right':self.model},motion_valid=lambda token:token=='owned',
            snapshot=self.backend.controller.snapshot,clock=lambda:1.,
            config={'arms':{'right':{'gripper_limits_raw':[-3.14,0.]}}})
        step=prepare_step('execute_pick',result,controller,'owned',policies={'pick':settings['pick']},
                          pick_setup=self.setup,preflight=lambda *args:True)
        self.assertEqual(step.monitor.arm_policy['tolerance'],settings['pick']['arm_policy']['tolerance'])
        self.assertNotIn('refinement',result)


if __name__=='__main__':unittest.main()
