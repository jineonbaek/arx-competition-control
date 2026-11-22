"""Fast competition routing with inert robot, vision and navigation adapters."""
import math
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import test_competition_task as fixtures
from battery_task import BatteryTask
from competition_modules import CarryReceiptModule, GraspModule
from agent.fast_pick import FAST_PREGRASP_JOINTS_DEG, enter_pregrasp_refinement
from agent.battery_extract import TRANSPORT_JOINTS_DEG
from agent.battery_flow import BatteryFlow
from agent.storage import RunStore
from agent.tests.test_battery_flow import Tools, review


class FastCompetitionTests(unittest.TestCase):
    def fixture(self):
        f=fixtures.CompetitionTests();f.setUp();self.addCleanup(f.doCleanups)
        return f

    def test_fast_pregrasp_then_transport_overlaps_navigation_and_lowering(self):
        f=self.fixture()
        grasp=SimpleNamespace(task=SimpleNamespace(lock=threading.RLock()),runs={})
        def picked(context):
            grasp.runs[context['request_id']]=dict(run_id='pick',flow={'status':'completed'},
                store=SimpleNamespace(state={'mission':{'stage':'EXTRACTED'}}))
            self.assertEqual(context['competition_mode'],'fast')
            self.assertGreater(context['pregrasp_completed_at'],0)
        f.modules['pick'].on_start=picked
        f.task.register('carry',CarryReceiptModule(grasp))
        f.modules['navigate_place'].status='running'
        arm=f.adapter.arm
        def hold_transport(side,joints,*args,**kwargs):
            if side=='right' and list(joints)==[math.radians(v) for v in TRANSPORT_JOINTS_DEG]:
                f.adapter.calls.append(('transport_submitted',list(joints)))
            else:arm(side,joints,*args,**kwargs)
        f.adapter.arm=hold_transport
        observed=[]
        def navigation(context):
            snapshot=f.controller.snapshot()
            observed.append(snapshot)
            self.assertEqual(snapshot['arms']['right']['joints'],[math.radians(v) for v in FAST_PREGRASP_JOINTS_DEG])
            self.assertEqual(snapshot['body']['height'],5.)
            self.assertIn(('lift',(0.,)),f.adapter.calls)
            self.assertTrue(any(c[0]=='transport_submitted' for c in f.adapter.calls))
        f.modules['navigate_place'].on_start=navigation
        f.task.action('browser',dict(action='start',request_id='fast',mode='fast',
            expected_epoch=f.controller.command_epoch,environment_clear=True))
        deadline=time.monotonic()+3
        while not observed and time.monotonic()<deadline:time.sleep(.01)
        self.assertTrue(observed,str(f.task.snapshot()))
        self.assertEqual(f.task.snapshot()['mode'],'fast')
        commands=[c for c in f.adapter.calls if c[0]=='arm']
        self.assertEqual(commands[1][1][1],[math.radians(v) for v in FAST_PREGRASP_JOINTS_DEG])
        self.assertEqual(f.controller.snapshot()['right_avoidance_preset']['tolerance_deg'],2.)
        f.action('stop');f.wait('paused')
        self.assertEqual(f.adapter.calls.count(('lift',(0.,))),1)
        self.assertFalse(f.task.snapshot()['control_owned'])
        self.assertFalse(f.controller.snapshot()['motion_reserved'])

    def test_unknown_mode_rejected_before_reserving_or_sending(self):
        f=self.fixture()
        with self.assertRaises(ValueError):
            f.task.action('browser',dict(action='start',request_id='bad',mode='other',
                expected_epoch=f.controller.command_epoch,environment_clear=True))
        self.assertFalse(f.controller.snapshot()['competition_reserved'])
        self.assertEqual(f.adapter.calls,[])

    def test_transport_ack_failure_cannot_be_replayed(self):
        grasp=SimpleNamespace(task=SimpleNamespace(lock=threading.RLock()),runs={
            'run:pick':dict(run_id='pick',flow={'status':'completed'},
                store=SimpleNamespace(state={'mission':{'stage':'EXTRACTED'}}))})
        module=CarryReceiptModule(grasp);controller=Mock()
        controller.submit.side_effect=TimeoutError('ack lost')
        context=dict(run_id='run',request_id='run:carry',competition_mode='fast',controller=controller,token='owned')
        with self.assertRaises(TimeoutError):module.start(context)
        self.assertEqual(module.inspect(context)['status'],'unknown')
        self.assertEqual(module.start(context)['status'],'unknown')
        controller.submit.assert_called_once()

    def test_owned_fast_settings_do_not_change_original_settings_or_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            controller=Mock();controller.motion_valid.return_value=True
            factory=Mock();factory.task_settings=dict(vision_mode='plan_f',lift_target_rad=12.52,
                lift_acceptance_rad=12.,observer_z_mm=700.5)
            task=BatteryTask(Path(directory)/'runs',controller,factory)
            original=task.task_settings.copy()
            with patch.object(task,'_start',return_value={}) as start:
                task.start_owned('token',True,fast_pregrasp_at=123.)
                fast=start.call_args.args[3]
                self.assertEqual(fast['pickup_flow'],'competition_fast')
                self.assertEqual(fast['recognition_profile'],original['recognition_profile'])
                task.start_owned('token',True)
                self.assertEqual(start.call_args.args[3],original)
            self.assertEqual(task.task_settings,original)

    def test_fast_starts_with_fresh_wrist_refinement_and_finishes_at_extraction(self):
        with tempfile.TemporaryDirectory() as directory:
            store=RunStore.create(directory);self.addCleanup(store.close)
            store.state['battery_task_settings']={'pickup_flow':'competition_fast','vision_mode':'plan_f'}
            tools=Tools(store);tools.vision_mode='d435'
            flow=BatteryFlow(tools,vision_mode='plan_f',review_plan=review)
            enter_pregrasp_refinement(flow,123.)
            self.assertEqual(flow.flow['phase'],'wrist')
            self.assertTrue(store.state['mission']['point3_refinement'])
            self.assertEqual(store.state['mission']['evidence_barriers']['PICK_AT_PREGRASP']['completed_at'],123.)
            self.assertEqual(flow.run()['status'],'completed')
            self.assertEqual(tools.calls,['observe_battery','preview_battery_grasp','plan_battery_pick',
                'execute_pick','verify_hold','stow_for_carry'])
            self.assertEqual(store.state['mission']['stage'],'EXTRACTED')

    def test_competition_runner_enters_refinement_without_moving_to_pregrasp_again(self):
        task=object.__new__(BatteryTask);task.lock=threading.RLock()
        task._prepare_plan_f_lift=Mock();task._restore_depth=Mock()
        store=Mock();store.state={'battery_task_settings':{'vision_mode':'plan_f',
            'pickup_flow':'competition_fast','fast_pregrasp_completed_at':123.}}
        flow=Mock();flow.run.return_value={'status':'completed'}
        state=dict(store=store,token='owned',owns_motion=False,run_id='fast')
        with patch('agent.fast_pick.enter_pregrasp_refinement') as enter,patch('agent.fast_pick.prepare_pregrasp') as move:
            task._run_task(state,flow)
        enter.assert_called_once_with(flow,123.);move.assert_not_called()
        flow._next.assert_called_once_with('fast_center')
        task._prepare_plan_f_lift.assert_called_once_with(state,flow)
