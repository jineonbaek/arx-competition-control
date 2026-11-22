"""Boolean routing, unchanged refinement, and durable close-only execution."""
import copy
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from PIL import Image

from agent.battery_backend import BatteryControllerBackend
from agent.battery_flow import BatteryFlow
from agent.battery_tools import BatteryExecutionTools
from agent.config import Config
from agent.contracts import AgentError
from agent.fast_center import judge_handle,CenteredCloseStep,CenteredCloseBackend
from agent.fast_pick import enter_pregrasp_refinement
from agent.mission import Mission
from agent.storage import RunStore
from agent.tests.test_battery_flow import Tools,review
from agent.tests.test_execution_policy import policy


class CenterTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.store=RunStore.create(temp.name);self.addCleanup(self.store.close)
        self.store.state['battery_task_settings']={'pickup_flow':'competition_fast','vision_mode':'plan_f'}

    def test_true_closes_without_detection_pose_or_arm_motion_then_extracts(self):
        tools=Tools(self.store);flow=BatteryFlow(tools,vision_mode='plan_f',review_plan=review)
        enter_pregrasp_refinement(flow,123.);flow._next('fast_center')
        def judgment(_):
            self.store.state['fast_handle_center']={'observation_id':'rgb','centered':True}
            return True
        with patch('agent.fast_center.judge_handle',side_effect=judgment):
            self.assertEqual(flow.run()['status'],'completed')
        self.assertEqual(tools.calls,['close_centered_handle','verify_hold','stow_for_carry'])
        self.assertEqual(self.store.state['mission']['stage'],'EXTRACTED')
        self.assertEqual(self.store.state['mission']['plan']['kind'],'stationary_handle_close')

    def test_false_continues_full_existing_refinement(self):
        tools=Tools(self.store);tools.corrections=1
        flow=BatteryFlow(tools,vision_mode='plan_f',review_plan=review)
        enter_pregrasp_refinement(flow,123.);flow._next('fast_center')
        with patch('agent.fast_center.judge_handle',return_value=False):
            self.assertEqual(flow.run()['status'],'completed')
        self.assertEqual(tools.calls[0],'observe_battery')
        self.assertEqual(tools.calls.count('execute_battery_approach'),2)
        self.assertNotIn('close_centered_handle',tools.calls)
        self.assertEqual(tools.calls[-3:],['execute_pick','verify_hold','stow_for_carry'])

    def test_model_failure_and_cancel_never_close(self):
        for cancel in (False,True):
            with self.subTest(cancel=cancel):
                self.store.state.pop('battery_flow',None)
                tools=Tools(self.store);flow=BatteryFlow(tools,vision_mode='plan_f',review_plan=review)
                flow._next('fast_center')
                def judgment(_):
                    if not cancel:raise AgentError('model_protocol_error','bad reply')
                    self.store.state['fast_handle_center']={'observation_id':'rgb','centered':True}
                    (self.store.path/'cancel.flag').touch()
                    return True
                with patch('agent.fast_center.judge_handle',side_effect=judgment):
                    self.assertEqual(flow.run()['status'],'cancelled' if cancel else 'needs_attention')
                self.assertEqual(tools.calls,[])

    def test_stable_flow_never_asks_center_and_does_not_expose_close(self):
        self.store.state['battery_task_settings']={'vision_mode':'plan_f'}
        tools=Tools(self.store);flow=BatteryFlow(tools,vision_mode='plan_f',review_plan=review)
        with patch('agent.fast_center.judge_handle') as judge:
            self.assertEqual(flow.run()['status'],'completed');judge.assert_not_called()
        self.assertEqual(tools.calls[-1],'return_to_carry')
        registry=BatteryExecutionTools(Config(),self.store)
        self.assertNotIn('close_centered_handle',registry.allowed_tools)

    def test_qwen_has_only_boolean_schema_and_references_2_4_and_current(self):
        path=self.store.path/'test.jpg';Image.new('RGB',(32,24)).save(path)
        tools=SimpleNamespace(store=self.store,config=Config(),observations=Mock())
        tools.observations.capture.return_value={'observation_id':'rgb'}
        reply=lambda value:{'tool_calls':[{'function':{'name':'report_handle_center','arguments':json.dumps(value)}}]}
        original=copy.deepcopy(tools.config.profiles)
        with patch.object(self.store,'image_path',return_value=path),patch('agent.fast_center.load_private_env'),\
                patch('agent.fast_center.reference_image',return_value=Image.new('RGB',(10,10))) as reference,\
                patch('agent.fast_center.QwenClient') as client:
            for value in (True,False):
                client.return_value.complete.return_value=reply({'centered':value})
                self.assertIs(judge_handle(tools),value)
            args,kwargs=client.return_value.complete.call_args
            self.assertEqual(kwargs,dict(parallel_tool_calls=False,tool_name='report_handle_center'))
            self.assertEqual(list(args[1][0]['function']['parameters']['properties']),['centered'])
            self.assertEqual([c.args[0] for c in reference.call_args_list],['d405-02','d405-04']*2)
            self.assertEqual([c['text'] for c in args[0][1]['content'] if 'text' in c][1:],
                             ['REFERENCE 图2','REFERENCE 图4','CURRENT'])
            self.assertFalse(client.call_args.args[0].profiles['cloud']['enable_thinking'])
            for bad in ({'centered':'true'},{'centered':True,'reason':'extra'},{}):
                client.return_value.complete.return_value=reply(bad)
                with self.assertRaises(AgentError):judge_handle(tools)
        self.assertEqual(tools.config.profiles,original)

    def test_close_reuses_gripper_policy_and_never_submits_arm_path(self):
        controller=Mock();controller.clock.return_value=10.;controller.motion_valid.return_value=True
        controller.submit.return_value={'ok':True}
        settings=policy()['pick'];before=copy.deepcopy(settings)
        step=CenteredCloseStep(controller,'owned',settings)
        self.assertEqual(step.start()['status'],'closing')
        controller.snapshot.return_value={'arms':{'right':dict(fresh=True,gripper=-1.,received=10.)},
            'capabilities':{'right:gripper':''}}
        with patch.object(step.arrival,'update',return_value=dict(arrived=False,status='stalled',stop_required=True)) as monitor:
            self.assertEqual(step.poll()['status'],'awaiting_hold_verification')
            monitor.assert_called_once_with([-1.],10.,10.,ready=True)
        controller.submit.assert_called_once_with({'type':'gripper','arm':'right','position':settings['gripper_target']},'owned')
        self.assertEqual(settings,before)
        with self.assertRaises(ValueError):step.start()

    def test_close_ack_loss_is_unknown_and_stops(self):
        controller=Mock();controller.clock.return_value=10.;controller.submit.return_value={'ok':False}
        receipt=CenteredCloseStep(controller,'owned',policy()['pick']).start()
        self.assertEqual(receipt['status'],'execution_unknown');self.assertTrue(receipt['stop_required'])
        controller.interrupt_motion.assert_called_once_with('owned')

    def test_new_receipt_does_not_claim_arm_arrival_and_requires_closure(self):
        mission=Mission({'stage':'FAST_HANDLE_CENTERED'})
        with self.assertRaises(AgentError):mission.apply('close_centered_handle',{}, {})
        receipt=CenteredCloseBackend._completed_data({})
        self.assertNotIn('pose_reached',receipt)
        mission.apply('close_centered_handle',receipt,{})
        self.assertEqual(mission.stage,'PICK_EXECUTED')

    def test_real_registry_backend_closes_once_and_produces_hold_receipt(self):
        controller=Mock();controller.motion_valid.return_value=True
        controller.clock.return_value=10.
        controller.snapshot.return_value={'arms':{'right':dict(fresh=True,gripper=-1.,received=10.,joints=[0.]*6)},
                                          'body':dict(fresh=True,height=12.52,received=10.)}
        backend=BatteryControllerBackend(controller,'owned',self.store,policies={'pick':policy()['pick']},
            pick_setup={},preflight=lambda *a:True,cancelled=lambda:False,timeout_s=policy()['timeout_s'])
        self.addCleanup(lambda:backend.cancel_and_wait(2))
        tools=BatteryExecutionTools(Config(motion_enabled=True),self.store,backends=backend)
        self.store.state['fast_handle_center']={'observation_id':'rgb','centered':True}
        self.store.state['mission'].update(stage='FAST_HANDLE_CENTERED',target={'target_id':'rgb'},
            plan=dict(plan_id='close',arm='right',kind='stationary_handle_close',observation_id='rgb'))
        step=Mock();step.start.return_value={'status':'awaiting_hold_verification','stop_required':False}
        flow=BatteryFlow(tools,vision_mode='plan_f',review_plan=review)
        with patch('agent.battery_backend.CenteredCloseStep',return_value=step) as prepare:
            flow._call('close_centered_handle',{'plan_id':'close'})
            flow._call('close_centered_handle',{'plan_id':'close'})
            prepare.assert_called_once();step.start.assert_called_once()
        self.assertEqual(self.store.state['mission']['stage'],'PICK_EXECUTED')
        flow._next('verify_hold');flow._call('verify_hold',{})
        self.assertEqual(self.store.state['mission']['stage'],'HOLD_CONFIRMED')
        self.assertIsNone(self.store.state['pending'])
