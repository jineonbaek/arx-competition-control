import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from pathlib import Path

import numpy as np
from agent.fast_pick import handle_support
from agent.battery_flow import BatteryFlow
from agent.tests.test_battery_flow import Tools, review
from agent.storage import RunStore
from fast_grasp_dock import approach
from fast_grasp_start import capture, check_start


class FastDockTests(unittest.TestCase):
    def simulate(self, *, cancel_after=10., lost_pose=False, send_error=False, invalid_origin=False):
        state = self.state = SimpleNamespace(t=0., x=0., moving=False, ends=0, commands=[])
        def sleep(dt):
            state.t += max(dt, 1e-8)
            if state.moving: state.x += .1 * dt
        def send(command, ttl_s):
            state.commands.append((command,ttl_s))
            state.moving = True
            if send_error: raise TimeoutError('ack')
        def stop():
            state.moving = False
            state.ends += 1
        def read():
            if lost_pose and state.commands: return None
            p = np.array([state.x,0.,0.])
            return p,p,state.t
        def check_origin():
            if invalid_origin: raise ValueError('起点')
        reader=SimpleNamespace(read=read,now=lambda:state.t,check_motion=lambda *args:None)
        output=SimpleNamespace(send=send,end_output=stop)
        return approach(reader,output,.225,.2,lambda:state.t>=cancel_after,
                        check_origin,clock=lambda:state.t,sleep=sleep)
    def test_one_t1_pulse_then_distinct_stopped_feedback(self):
        result=self.simulate()
        self.assertEqual(result['source'],'t1_short_pulse')
        self.assertEqual(self.state.commands,[((.225,0.,0.),.2)])
        self.assertAlmostEqual(self.state.x,.020)
        self.assertEqual(self.state.ends,1)
        self.assertGreaterEqual(self.state.t,2.2)
    def test_wrong_origin_has_no_output(self):
        with self.assertRaisesRegex(ValueError,'起点'):self.simulate(invalid_origin=True)
        self.assertEqual(self.state.commands,[])
    def test_missing_ack_is_ended_and_never_replayed(self):
        with self.assertRaises(TimeoutError):self.simulate(send_error=True)
        self.assertEqual(len(self.state.commands),1)
        self.assertEqual(self.state.ends,1)
    def test_lost_pose_and_cancellation_end_output(self):
        for kwargs in ({'lost_pose':True},{'cancel_after':1.1},{'cancel_after':1.6}):
            with self.assertRaises(ValueError):self.simulate(**kwargs)
            self.assertEqual(self.state.ends,1)
            self.assertFalse(self.state.moving)


class FastPlanTests(unittest.TestCase):
    def setUp(self):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        self.store=RunStore.create(directory.name);self.addCleanup(self.store.close)
        self.store.state['battery_task_settings']={'pickup_flow':'fast'}
        self.store.state['fast_dock']={'run_id':self.store.state['run_id'],'status':'stopped','stopped_at':100.,'source':'t1_short_pulse'}
        self.tools=Tools(self.store)
        # Model the planner's direct PICK_PLANNED response and record every sensor requested.
        self.tools.vision_mode='d435'
        self.cameras=[]
        original=self.tools.execute
        def execute(name,args,ident):
            if name=='observe_battery':self.cameras.append(args['camera'])
            return original(name,args,ident)
        self.tools.execute=execute
    def run_fast(self):
        return BatteryFlow(self.tools,vision_mode='plan_f',review_plan=review).run()
    def test_fast_confirms_one_wrist_frame_closes_once_and_ends_at_retreat(self):
        self.assertEqual(self.run_fast()['status'],'completed')
        self.assertEqual(self.cameras,['d405'])
        self.assertEqual(self.tools.calls,['observe_battery','preview_battery_grasp','plan_battery_pick',
                                          'execute_pick','verify_hold','stow_for_carry'])
    def test_foreign_receipt_prevents_observation_and_pick(self):
        self.store.state['fast_dock']['run_id']='old'
        self.assertEqual(self.run_fast()['status'],'needs_attention')
        self.assertEqual(self.tools.calls,[])
    def test_unconfirmed_hold_never_backs_up(self):
        self.tools.fail='verify_hold'
        self.assertEqual(self.run_fast()['status'],'needs_attention')
        self.assertNotIn('stow_for_carry',self.tools.calls)

    def test_replacement_fast_entry_reuses_plan_f_and_finishes_transport(self):
        self.store.state['battery_task_settings']={'pickup_flow':'fast_plan_f'}
        self.store.state.pop('fast_dock',None)
        self.tools.vision_mode='plan_f'
        self.assertEqual(self.run_fast()['status'],'completed')
        self.assertIn('execute_battery_approach',self.tools.calls)
        self.assertEqual(self.tools.calls[-3:],['verify_hold','stow_for_carry','return_to_carry'])
        self.assertTrue(all(camera=='d405' for camera in self.cameras))
    def test_body_or_one_side_is_not_a_confirmed_bridge(self):
        profile=dict(points_object_mm={'front':[0,0,195]},hole_bounds_mm=[[-20,0,162],[20,0,177]],handle_thickness_mm=24)
        pts=np.zeros((6,4,3));pts[:,:,2]=.19;pts[:,:2,0]=-.01;pts[:,2:,0]=.01
        frame={'xyz':pts,'id':'w1'};mask=np.ones((6,4));pose={'T_camera_from_object':np.eye(4)}
        self.assertTrue(handle_support(frame,mask,pose,profile)['confirmed'])
        mask[:,:2]=0
        self.assertFalse(handle_support(frame,mask,pose,profile)['confirmed'])
        mask[:]=1;pts[:,:,2]=.15
        self.assertFalse(handle_support(frame,mask,pose,profile)['confirmed'])
    def test_saved_reference_is_map_specific(self):
        nav={'pose':dict(valid=True,frame='map',x=3,y=2,yaw=.1,age_s=.2),
             'route':{'definition':{'map_id':'a'}}}
        arm=dict(fresh=True,joints=[0]*6,gripper=0)
        robot=dict(chassis_active=False,body=dict(fresh=True,height=3),arms=dict(left=arm,right=arm))
        ref=capture(robot,nav);check_start(ref,nav)
        nav['route']['definition']['map_id']='b'
        with self.assertRaises(ValueError):check_start(ref,nav)


class FastEntryTests(unittest.TestCase):
    def test_fast_and_stable_receive_identical_plan_f_parameters(self):
        from battery_task import BatteryTask
        with tempfile.TemporaryDirectory() as directory:
            factory=Mock()
            factory.task_settings=dict(vision_mode='plan_f',lift_target_rad=12.52,
                lift_acceptance_rad=12.,observer_z_mm=700.5)
            task=BatteryTask(Path(directory)/'runs',Mock(),factory)
            profile=task.task_settings['recognition_profile']
            with patch.object(task,'_start',return_value={'status':'idle'}) as start:
                for action in ['start','start_fast']:
                    task.action('owner',dict(action=action,request_id=action,expected_epoch=4,
                        environment_clear=True,lift_target_rad=12.52,observer_z_mm=700.5,
                        vision_mode='plan_f',recognition_profile=profile))
                stable=start.call_args_list[0].args[3]
                fast=start.call_args_list[1].args[3].copy()
            self.assertEqual(fast.pop('pickup_flow'),'fast_plan_f')
            self.assertEqual(fast,stable)

    def test_stable_entry_never_uses_fast_pregrasp(self):
        from battery_task import BatteryTask
        from threading import RLock
        task=object.__new__(BatteryTask);task.lock=RLock()
        task._prepare_plan_f_lift=Mock();task._restore_depth=Mock()
        store=Mock();store.state={'battery_task_settings':{'vision_mode':'plan_f'}}
        flow=Mock();flow.run.return_value={'status':'completed'}
        state={'store':store,'token':'owned','owns_motion':False,'run_id':'stable'}
        with patch('agent.fast_pick.prepare_pregrasp') as prepare:
            task._run_task(state,flow)
        prepare.assert_not_called()
        task._prepare_plan_f_lift.assert_called_once_with(state,flow)
        flow.run.assert_called_once()

    def test_replacement_fast_uses_plan_f_settings_without_docking_or_map_reference(self):
        from battery_task import BatteryTask
        import threading
        import json
        with tempfile.TemporaryDirectory() as d:
            ref=Path(d)/'ref.json'
            ref.write_text(json.dumps(dict(map_id='map-a',pose=[3.,2.,0.])))
            controller=Mock()
            controller.snapshot.return_value={'command_epoch':4,'chassis_active':False}
            controller.motion_valid.return_value=True
            controller.reserve_motion.return_value='token'
            stop=threading.Event()
            dock=Mock(policy='t1_short_pulse')
            dock.return_value=dict(status='stopped',source='t1_short_pulse',stopped_at=4.)
            def build(store,*args):
                def run():
                    stop.wait(1.)
                    return dict(status='completed')
                return SimpleNamespace(run=run)
            task=BatteryTask(Path(d)/'runs',controller,build,fast_start_path=ref,fast_dock=dock)
            selected=task.task_settings['recognition_profile'].copy()
            ref.unlink()
            request=dict(action='start_fast',request_id='request',expected_epoch=4,environment_clear=True)
            view={'pose':dict(valid=True,frame='map',x=3.,y=2.,yaw=0.),
                  'route':{'definition':{'map_id':'map-a'}}}
            with patch('navigation_view.read_snapshot',return_value=view) as navigation,patch('agent.fast_pick.prepare_pregrasp') as prepare,patch.object(task,'_prepare_plan_f_lift') as lift:
                try:
                    first=task.action('owner',request)
                    self.assertEqual(task.action('owner',request)['run_id'],first['run_id'])
                    self.assertEqual(first['settings']['recognition_profile'],selected)
                    self.assertEqual(first['settings']['pickup_flow'],'fast_plan_f')
                    self.assertEqual(first['settings']['vision_mode'],'plan_f')
                    self.assertNotIn('pickup_flow',task.task_settings)
                finally:
                    stop.set()
                    task.active['worker'].join(3.)
                    task.close()
            dock.assert_not_called();prepare.assert_not_called();navigation.assert_not_called()
            lift.assert_called_once_with(task.active,task.active['runner'])
            self.assertFalse(controller.submit.called)

    def test_direct_pick_opens_for_approach_then_accepts_closing_feedback(self):
        from test_battery_step_preparation import PreparationTests
        from agent.gripper_open import OpenThenApproachStep
        from agent.pick_execution import PickTrajectoryStep
        fixture=PreparationTests()
        fixture.setUp()
        fixture.plan.update(vision_mode='d405',pickup_flow='fast',observation_id='obs',
                            snapshot_id='frame',refinement={})
        step=fixture.prepare('execute_pick')
        self.assertIsInstance(step,OpenThenApproachStep)
        self.assertIsInstance(step.approach,PickTrajectoryStep)
        with patch.object(step,'_open',return_value=False):
            step.preflight=lambda *args:True
            self.assertFalse(step._approach_ready({},0.))
            step.monitor.stage='closing'
            self.assertTrue(step._approach_ready({},0.))
