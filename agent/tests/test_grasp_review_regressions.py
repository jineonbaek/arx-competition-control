"""Final review regressions, fake transports only."""
import unittest,time,copy
import numpy as np
from unittest.mock import patch
from agent.tests import test_grasp_pose_recovery as f
from agent.tests import test_grasp_relaxed_arrival as a
from agent.grasp_recovery import GraspRecovery
from agent.battery_flow import BatteryFlow
from agent.tests.test_battery_flow import Tools,review

class RecoveryFixTests(unittest.TestCase):
    setUp=f.RecoveryTests.setUp
    def test_reference_complete_without_moving_is_not_posture_success(self):
        self.recovery.capture();base=self.c.snapshot
        def snapshot():self.c.now+=.05;return base()
        self.c.snapshot=snapshot
        self.c.start_trajectory=lambda *args,**kw:None
        self.c.poll_trajectory=lambda *args,**kw:{'status':'reference_complete'}
        with patch('agent.grasp_recovery.time.sleep'):
            with self.assertRaises(ValueError):self.recovery.recover('vision_failed')
        self.assertEqual(self.recovery.state['status'],'unknown')
    def test_older_recovery_request_is_not_replayed(self):
        self.recovery.capture()
        def move(points):self.c.q=list(points[-1]);return points
        with patch.object(self.recovery,'_move',side_effect=move) as moving:
            self.recovery.recover('vision_failed','A');self.recovery.recover('vision_failed','B')
            count=moving.call_count
            try:self.recovery.recover('vision_failed','A')
            except ValueError:pass
            self.assertEqual(moving.call_count,count)
    def test_duplicate_approach_receipt_does_not_add_another_roundtrip(self):
        self.recovery.capture();via=list(self.c.q);via[1]+=.2;end=via.copy();end[0]+=.2
        data={'execution_id':'approach1','return_trace_rad':[list(self.c.q),via,end]}
        self.recovery.record_approach(data);before=copy.deepcopy(self.recovery.state['outbound_trace'])
        self.recovery.record_approach(data)
        self.assertEqual(self.recovery.state['outbound_trace'],before)
    def test_cannot_capture_new_origin_for_preexisting_midrun(self):
        self.store.state['battery_flow']={'phase':'preview','index':4}
        self.store.state['mission']['approach_plan']={'old':True}
        with self.assertRaises(ValueError):self.recovery.capture()
    def test_failed_stop_retains_real_controller_owner_after_reference_complete(self):
        from control import Controller,Rejected
        from test_control import configured,FakeAdapter
        c=Controller(configured(),FakeAdapter(),lambda:10.)
        for side in ('left','right'):c.update_arm(side,[0.]*6,-3.14,[0.]*6)
        c.update_body(12.,[0.]*4,0,0,0);c.wrist_reason=lambda side:''
        competition=c.reserve_competition(c.command_epoch)
        token=c.reserve_motion(c.command_epoch,competition=competition)
        self.b.controller=c;self.b.token=token
        recovery=GraspRecovery(self.b,environment_clear=True);recovery.capture()
        with patch.object(recovery,'_validate_points',return_value=100.),patch.object(recovery,'_move',side_effect=ValueError('not settled')),patch.object(c,'submit',side_effect=Rejected('stop failed')):
            with self.assertRaises(ValueError):recovery.recover('vision_failed')
        with self.assertRaises(Rejected):c.release_motion(token)
        with self.assertRaises(Rejected):c.release_competition(competition)
        self.assertTrue(c.snapshot()['motion_reserved'])
    def test_repeated_correction_without_progress_selects_pose_recovery(self):
        tools=Tools(self.store);tools.vision_mode='plan_f';tools.corrections=3
        from types import SimpleNamespace
        r=SimpleNamespace(capture=lambda:None,record_approach=lambda data:True,
                          recover=lambda reason,**kw:{'completed_at':time.time()})
        with patch.object(r,'record_approach',side_effect=[True,False,True,True,True,True]),patch.object(r,'recover',wraps=r.recover) as recovering:
            out=BatteryFlow(tools,review_plan=review,vision_mode='plan_f',recovery=r).run()
        self.assertEqual(out['status'],'completed');self.assertEqual(recovering.call_count,1)

class PairedFeedbackTests(unittest.TestCase):
    def test_new_arm_motion_with_unchanged_lift_stamp_cannot_close(self):
        fixture=a.RelaxedTests();g=fixture.make();g.begin_settling(10.)
        for t in (10.01,10.07,10.13):g.update(fixture.state(t),t,ready=True)
        moved=fixture.state(10.19,.025);moved['body']['received']=10.13
        g.update(moved,10.19,ready=True)
        moved=fixture.state(10.20,.025);moved['body']['received']=10.14
        self.assertFalse(g.start_closing(moved,10.20,ready=True)['close_permitted'])
    def test_actual_start_connector_is_checked_before_gripper_opens(self):
        import test_battery_step_preparation as fixture
        t=fixture.PreparationTests();t.setUp();t.plan['arrival_mode']='reference_settled'
        t.f.q[0]+=.1;t.f.feedback(10.01,grip=-3.14);actual=t.f.q.copy();seen=[]
        def check(start,end,lift):
            seen.append((np.array(start),np.array(end)))
            return {'clear':not np.allclose(start,actual),'lower_mm':100.}
        with patch.object(t.c.models['right'],'check',side_effect=check):
            step=t.prepare();receipt=step.start()
        self.assertTrue(receipt['stop_required'])
        self.assertTrue(any(np.allclose(start,actual) for start,end in seen))

class PreviewTimeTests(unittest.TestCase):
    def test_recovery_barrier_is_required_for_preview_and_plan_consumers(self):
        from agent.battery_tools import BatteryExecutionTools
        from agent.config import Config
        from agent.storage import RunStore
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            store=RunStore.create(directory)
            try:
                now=time.time();store.state['battery_task_settings']={'vision_mode':'plan_f'}
                obs=store.save_observation({'captured_at':now-1,'cameras':{'d405':{'captured_at':now-1,'image_size_px':[640,480]}}},{})['observation_id']
                tools=BatteryExecutionTools(Config(),store)
                for name,args in [('preview_battery_grasp',{'observation_id':obs,'clearance_mm':50.}),('plan_battery_pick',{'observation_id':obs})]:
                    store.state['mission'].update(stage='COARSE_ALIGNED',evidence_barriers={'GRASP_RECOVERY':{
                        'completed_at':now,'previous_observation_ids':[]}})
                    with patch.object(tools,'_perception',return_value={'status':'preview_complete'}),patch.object(tools,'_localized',return_value=obs):
                        r=tools.execute(name,args,name)
                    self.assertFalse(r['ok']);self.assertEqual(r['error']['code'],'post_action_evidence_required')
            finally:store.close()
