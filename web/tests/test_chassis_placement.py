"""Saved special point + existing six-second chassis push, never an arm stroke."""
import copy
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import test_lidar_fixed_stroke as fixed
import test_navigation_placement as nav
import test_competition_task as scheduler


class PrepareTests(unittest.TestCase):
    def test_saved_special_endpoint_is_used_without_editing_coordinates(self):
        from chassis_placement import special_target
        definition={'points':[{'id':'t2','type':'task','point_id':'point_2','pose':[0.,0.,0.]},
                              {'id':'saved-7','type':'special','pose':[.174,.004,-.02]}]}
        before=copy.deepcopy(definition)
        result=special_target(definition)
        self.assertEqual(result['id'],'saved-7');self.assertEqual(result['pose'],[.174,.004,-.02])
        self.assertEqual(definition,before)
        with self.assertRaises(ValueError):special_target({'points':definition['points'][:1]})

    def test_arm_stops_at_observation_with_clamp_closed_and_no_insert_or_release(self):
        fx=fixed.AutomaticRouteTests();self.addCleanup(fx.doCleanups)
        factory=fixed.horizontal_fixtures.D405HorizontalFlowTests
        original=factory.make_level
        def make(instance):
            level=original(instance);level.driver='driver';level.raw=-1.;level.initial_q=level.ctrl.q.copy()
            return level
        with patch.object(factory,'make_level',make):
            state,events=fx.exercise('lidar_special_point','lidar_point_2')
        self.assertEqual(state['status'],'prepared')
        self.assertIn('j4',events)
        self.assertNotIn('push',events);self.assertNotIn('release_return',events)
        self.assertNotIn('camera_trim',events);self.assertNotIn('camera_seat',events)
        self.assertTrue(state['preobservation_ready']);self.assertFalse(state['released'])

    def test_navigation_preparation_is_explicitly_pending_placement(self):
        fx=nav.NavigationPlacementTests();fx.setUp();self.addCleanup(fx.doCleanups)
        fx.module.lidar_locked=True;fx.arm.placement_mode='lidar_special_point'
        fx.nav.visit.update(arrival_pose=[1.2,2.3,.1],goal_pose=[1.3,2.4,.2],pose_stamp_ns=123)
        fx.module._stop_chassis=lambda *a:self.fail('T2 must not repeat the navigation parking check')
        fx.module._pose=lambda:self.fail('T2 must not wait for another map pose sample')
        fx.module._observe=lambda *a:self.fail('T2 must not request visual chassis alignment')
        fx.nav.prepared_definition={'points':[{'id':'t2','type':'task','point_id':'point_2','pose':[0.,0.,0.]},{'id':'end','type':'special','pose':[.17,0.,0.]}]}
        fx.arm.snapshot=lambda:dict(status='prepared',run_id='arm-1',preobservation_ready=True,
            released=False,chassis_hold_joints_rad=[0.]*6,chassis_initial_joints_rad=[0.]*6,
            chassis_return_route_rad=[[0.]*6],chassis_driver_instance='driver',chassis_gripper_raw=-1.)
        fx.module.start(fx.context);result=fx.wait()
        self.assertEqual(result['status'],'completed',result)
        self.assertTrue(result['placement_pending']);self.assertFalse(result['placement_confirmed'])
        self.assertEqual(result['completion_basis'],'preobservation_only')
        self.assertEqual(result['chassis_start_pose'],dict(x=1.2,y=2.3,yaw=.1))
        handoff=fx.arm.starts[0][1]
        self.assertEqual(handoff['chassis_pose_source'],'navigation_arrival_receipt')
        self.assertEqual(handoff['navigation_visit_id'],fx.nav.visit['visit_id'])
        self.assertEqual(handoff['pose_stamp_ns'],123)


class ScheduleTests(unittest.TestCase):
    def exercise(self,forward_failure=False):
        fx=scheduler.CompetitionTests();fx.setUp();self.addCleanup(fx.doCleanups)
        active=[True];trace=fx.trace
        def route(context):trace.append(('route_end',));active[0]=False
        def forward(context):
            self.assertTrue(context['chassis_insertion'])
            trace.append(('forward_6s',))
            if forward_failure:raise ValueError('forward interrupted')
            return dict(status='completed',duration_s=6.,sdk_forward=.3,output_ended=True)
        navigation=SimpleNamespace(prepare=lambda ctx:None,snapshot=lambda:{},check=lambda:None,
            before_work=lambda *a:None,complete_route=route,finish_forward=forward,
            stop=lambda ctx:dict(stopped=True))
        class Navigation:
            active=property(lambda self:active[0])
            def __getattr__(self,name):return getattr(navigation,name)
        fx.task.navigation=Navigation()
        release=scheduler.Module(trace,'release_after_chassis')
        release.on_start=lambda ctx:self.assertEqual(ctx['forward_receipt']['duration_s'],6.)
        fx.task.modules['release_after_chassis']=release
        fx.action('start');view=fx.wait('needs_attention' if forward_failure else 'completed')
        return view,trace

    def test_route_then_six_seconds_then_release_once(self):
        view,trace=self.exercise()
        names=[x[1] if x[0]=='start' else x[0] for x in trace]
        self.assertLess(names.index('route_end'),names.index('forward_6s'))
        self.assertLess(names.index('forward_6s'),names.index('release_after_chassis'))
        self.assertEqual(names.count('release_after_chassis'),1)
        self.assertEqual(view['status'],'completed')

    def test_failed_six_second_push_never_releases(self):
        _,trace=self.exercise(True)
        self.assertNotIn(('start','release_after_chassis'),[x[:2] for x in trace])


class FinalizeTests(unittest.TestCase):
    def fixture(self,receipt=None):
        from chassis_placement import ChassisReleaseModule
        events=[];q=np.zeros(6);grip=[-1.]
        state=lambda:dict(arms={'right':dict(fresh=True,joints=q.tolist(),gripper=grip[0])},body=dict(fresh=True,height=0.),chassis_active=False)
        def submit(command,token,preflight=None):
            if preflight:preflight(state(),time.time())
            events.append('release');grip[0]=command['position'];return dict(ok=True)
        ctrl=SimpleNamespace(config={'arms':{'right':{'gripper_open_raw':-3.}}},snapshot=state,motion_valid=lambda token:True,submit=submit,
            adapter=SimpleNamespace(trajectory_status=lambda side:dict(driver_instance='driver',ready=True,enabled=True)),
            end_chassis_output=lambda token:events.append('stop_chassis'))
        prepared=dict(chassis_hold_joints_rad=q.tolist(),chassis_initial_joints_rad=q.tolist(),
            chassis_return_route_rad=[q.tolist()],chassis_driver_instance='driver',chassis_gripper_raw=-1.,
            preobservation_ready=True,released=False)
        backend=SimpleNamespace(save=lambda **v:events.append('save:'+v.get('phase','state')))
        module=SimpleNamespace(ctrl=ctrl,placement=backend,root=None,
            runs={'run:place':dict(placement=prepared,chassis_start_pose=dict(x=1.,y=2.,yaw=0.))})
        finalizer=ChassisReleaseModule(module)
        finalizer._retreat=lambda context,run,goal,guard:(guard(),events.append('retreat'),dict(status='completed'))[-1]
        finalizer._arm_return=lambda context,run,p:(events.append('arm_return'))
        context=dict(request_id='run:release_after_chassis',run_id='run',token=object(),cancelled=threading.Event(),controller=ctrl,
            forward_receipt=receipt if receipt is not None else dict(status='completed',duration_s=6.,sdk_forward=.3,output_ended=True))
        return finalizer,context,events

    def test_six_second_completion_releases_then_retracts_then_returns_arm(self):
        f,ctx,events=self.fixture();f.start(ctx);f.runs[ctx['request_id']]['thread'].join(2.)
        result=f.inspect(ctx);self.assertEqual(result['status'],'completed',result)
        self.assertLess(events.index('release'),events.index('retreat'));self.assertLess(events.index('retreat'),events.index('arm_return'))
        self.assertFalse(result['insertion_depth_verified']);self.assertTrue(result['returned'])
        with self.assertRaises(ValueError):f.start(ctx)

    def test_release_accepts_preparation_run_with_live_thread_handles(self):
        f,ctx,events=self.fixture()
        original=f.placement.runs['run:place']
        original.update(stop=threading.Event(),thread=threading.Thread())
        f.start(ctx);f.runs[ctx['request_id']]['thread'].join(2.)
        self.assertEqual(f.inspect(ctx)['status'],'completed')
        self.assertEqual(events.count('release'),1)
        self.assertLess(events.index('release'),events.index('retreat'))
        self.assertFalse(original['stop'].is_set())

    def test_missing_or_incomplete_forward_receipt_never_opens_gripper(self):
        for receipt in ({},dict(status='completed',duration_s=6.,sdk_forward=.5,output_ended=True),dict(status='completed',duration_s=4.,sdk_forward=.3,output_ended=True),
                        dict(status='completed',duration_s=5.99,sdk_forward=.3,output_ended=True),
                        dict(status='completed',duration_s=6.,sdk_forward=.3,output_ended=False)):
            f,ctx,events=self.fixture(receipt)
            with self.assertRaises(ValueError):f.start(ctx)
            self.assertNotIn('release',events)

    def test_cancel_before_release_holds_gripper(self):
        f,ctx,events=self.fixture();ctx['cancelled'].set()
        f.start(ctx);f.runs[ctx['request_id']]['thread'].join(2.)
        self.assertNotIn('release',events);self.assertNotEqual(f.inspect(ctx)['status'],'completed')

    def test_joint_drift_does_not_interrupt_chassis_handoff_or_release(self):
        f,ctx,events=self.fixture();original=f.ctrl.snapshot
        def drifted():
            state=original();state['arms']['right']['joints']=[0.,0.,0.,.1,0.,0.]
            return state
        f.ctrl.snapshot=drifted
        f.hold(ctx)
        self.assertNotIn('release',events)
        f.start(ctx);f.runs[ctx['request_id']]['thread'].join(2.)
        self.assertEqual(f.inspect(ctx)['status'],'completed')
        self.assertLess(events.index('release'),events.index('retreat'))

    def test_driver_or_gripper_loss_still_stops_chassis_handoff(self):
        for problem in ('driver','feedback','gripper'):
            with self.subTest(problem=problem):
                f,ctx,events=self.fixture();original=f.ctrl.snapshot
                if problem=='driver':
                    f.ctrl.adapter.trajectory_status=lambda side:dict(driver_instance='other',ready=True,enabled=True)
                else:
                    def failed_state():
                        state=original()
                        if problem=='feedback':state['arms']['right']['fresh']=False
                        else:state['arms']['right']['gripper']=-3.
                        return state
                    f.ctrl.snapshot=failed_state
                with self.assertRaises(ValueError):f.hold(ctx)
                self.assertNotIn('release',events)


class ActualForwardTests(unittest.TestCase):
    def test_existing_terminal_forward_runs_full_duration_then_ends_output(self):
        from navigation_session import NavigationSession
        now=[0.];sent=[];checks=[]
        class Cancel:
            def is_set(self):return False
            def wait(self,seconds):now[0]+=seconds;return False
        ctrl=SimpleNamespace(motion_valid=lambda token:True,snapshot=lambda:dict(body=dict(fresh=True),nav_sdk_ready=True),submit=lambda *a:None)
        transport=SimpleNamespace(send=lambda values,**kw:sent.append(values),end_output=lambda:checks.append('stopped'))
        session=object.__new__(NavigationSession)
        context=dict(controller=ctrl,token=object(),cancelled=Cancel(),chassis_insertion=True,chassis_hold=lambda:checks.append('held'))
        result=session.finish_forward(context,clock=lambda:now[0],transport_factory=lambda **kw:transport)
        self.assertGreaterEqual(result['duration_s'],6.)
        self.assertLess(result['duration_s'],6.05)
        self.assertEqual(result['sdk_forward'],.3)
        self.assertTrue(result['output_ended']);self.assertEqual(checks[-1],'stopped')
        self.assertTrue(sent);self.assertTrue(all(command==(.3,0.,0.) for command in sent))

    def test_legacy_terminal_forward_stays_four_seconds(self):
        from navigation_session import NavigationSession
        now=[0.];ended=[];sent=[]
        class Cancel:
            def is_set(self):return False
            def wait(self,seconds):now[0]+=seconds;return False
        ctrl=SimpleNamespace(motion_valid=lambda token:True,snapshot=lambda:dict(body=dict(fresh=True),nav_sdk_ready=True),submit=lambda *a:None)
        transport=SimpleNamespace(send=lambda command,**kw:sent.append(command),end_output=lambda:ended.append(True))
        context=dict(controller=ctrl,token=object(),cancelled=Cancel(),chassis_insertion=False)
        result=object.__new__(NavigationSession).finish_forward(context,clock=lambda:now[0],transport_factory=lambda **kw:transport)
        self.assertAlmostEqual(result['duration_s'],4.,places=6)
        self.assertEqual(ended,[True])
        self.assertEqual(result['sdk_forward'],.225)
        self.assertTrue(sent);self.assertTrue(all(command==(.225,0.,0.) for command in sent))

    def test_holding_failure_ends_chassis_output_without_a_completion_receipt(self):
        from navigation_session import NavigationSession
        ended=[]
        ctrl=SimpleNamespace(submit=lambda *a:None)
        transport=SimpleNamespace(send=lambda *a,**kw:self.fail('movement after hold failure'),end_output=lambda:ended.append(True))
        def broken():raise ValueError('arm hold lost')
        context=dict(controller=ctrl,token=object(),cancelled=threading.Event(),chassis_hold=broken)
        with self.assertRaisesRegex(ValueError,'hold lost'):
            object.__new__(NavigationSession).finish_forward(context,clock=lambda:0.,transport_factory=lambda **kw:transport)
        self.assertEqual(ended,[True])
