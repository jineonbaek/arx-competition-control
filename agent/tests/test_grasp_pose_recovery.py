"""Offline recovery routing; fake controller never reaches hardware."""
import copy
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from agent.storage import RunStore
from agent.contracts import AgentError
from agent.battery_flow import BatteryFlow
from agent.tests.test_battery_flow import Tools,review
from agent.tests.test_execution_policy import policy
from agent.grasp_recovery import GraspRecovery, POINT3_JOINTS_DEG, simplify_trace

class Controller:
    def __init__(self):
        self.now=10.;self.q=[0.,.2,.4,.1,0.,-1.57];self.grip=-3.14;self.lift=12.
        self.motion_trajectory=None;self.valid=True;self.calls=[];self.stop_pending=False
        self.models={'right':SimpleNamespace(check=lambda *a:dict(clear=True,lower_mm=100.),reach=1.,
            wrist_transform=lambda q,h:np.eye(4))}
        self.config={'arms':{'right':{'joint_limits_deg':[[-180,180]]*6}}}
    def clock(self):return self.now
    def motion_valid(self,t):return self.valid
    def snapshot(self):
        return dict(chassis_active=False,motion_interrupted=False,contact_stop_pending=self.stop_pending,
                    capabilities={'right:arm.wrist':'','right:gripper':''},
                    arms={'right':dict(joints=list(self.q),gripper=self.grip,received=self.now,fresh=True)},
                    body=dict(height=self.lift,received=self.now,fresh=True))
    def begin_contact_stop(self,t):self.stop_pending=True
    def interrupt_motion(self,t):self.valid=False
    def submit(self,command,token):self.calls.append(command);return {'ok':True}
    def log(self,*args):pass

class RecoveryTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        self.store=RunStore.create(tmp.name);self.addCleanup(self.store.close)
        self.c=Controller();p=policy()
        self.b=SimpleNamespace(controller=self.c,token='owned',store=self.store,policies=p,
            pick_setup={'timing_limits':p['timing_limits'],'T_wrist_from_tool':np.eye(4).tolist()},
            cancelled=lambda:False,directory=self.store.path/'executions',entries={})
        self.b.directory.mkdir()
        self.recovery=GraspRecovery(self.b,environment_clear=True)
    def test_origin_is_copied_once_and_never_placement_pose(self):
        self.store.state['placement_preobserve']={'joints_rad':[9]*6}
        original=list(self.c.q);self.recovery.capture()
        self.c.q[0]=.2;self.recovery.capture()
        self.assertEqual(self.store.state['grasp_recovery']['point2']['joints_rad'],original)
    def test_recovery_returns_along_completed_trace_then_moves_to_point3(self):
        self.recovery.capture();origin=list(self.c.q)
        via=np.array(origin);via[1]+=.05;end=via.copy();end[0]+=.04
        self.recovery.record_approach({'execution_id':'approach1','return_trace_rad':[origin,via.tolist(),end.tolist()]})
        self.c.q=end.tolist();routes=[]
        def move(points):
            routes.append(copy.deepcopy(points));self.c.q=list(points[-1]);return points
        with patch.object(self.recovery,'_move',side_effect=move):r=self.recovery.recover('preview_failed')
        self.assertEqual(routes[0][-1],origin)
        self.assertTrue(any(np.allclose(v,via) for v in routes[0]))
        np.testing.assert_allclose(np.degrees(routes[1][-1]),[15.,32.,45.,-13.,16.,-90.])
        self.assertEqual(self.c.grip,-3.14)
        self.assertEqual(self.c.lift,12.)
        self.assertEqual(r['attempt'],1)
        self.assertEqual(self.store.state['grasp_recovery']['status'],'ready')
    def test_unknown_hold_or_unknown_execution_or_cancel_never_returns(self):
        for fault in ('grip','pending','unknown','cancel','contact'):
            with self.subTest(fault=fault):
                self.store.state.pop('grasp_recovery',None);self.c.grip=-3.14
                self.store.state['pending']=None;self.recovery.capture()
                if fault=='grip':self.c.grip=-1.
                if fault=='pending':self.store.state['pending']={'unknown':True}
                if fault=='unknown':self.store.state['grasp_recovery']['status']='moving'
                if fault=='cancel':(self.store.path/'cancel.flag').touch()
                if fault=='contact':self.c.valid=False
                with patch.object(self.recovery,'_move') as move:
                    with self.assertRaises((ValueError,AgentError)):self.recovery.recover('vision_failed')
                    move.assert_not_called()
                self.c.valid=True
                (self.store.path/'cancel.flag').unlink(missing_ok=True)
    def test_unknown_prior_journal_and_post_closure_do_not_retry(self):
        self.recovery.capture()
        for fault in ('journal','closed'):
            if fault=='journal':(self.b.directory/'previous.json').write_text('{"receipt":{"status":"unknown"}}')
            else:
                (self.b.directory/'previous.json').unlink()
                self.store.state['mission']['stage']='PICK_EXECUTED'
            with patch.object(self.recovery,'_move') as move:
                with self.assertRaises((ValueError,AgentError)):self.recovery.recover('vision_failed')
                move.assert_not_called()
    def test_four_point3_retries_then_existing_bounded_stop(self):
        self.recovery.capture();postures=[]
        def move(points):self.c.q=list(points[-1]);return points
        with patch.object(self.recovery,'_move',side_effect=move):
            for i in range(4):postures.append(tuple(self.recovery.recover('vision_failed')['target_joints_rad']))
            with self.assertRaisesRegex(ValueError,'exhausted'):self.recovery.recover('vision_failed')
        self.assertEqual(len(postures),4)
        for q in postures:np.testing.assert_allclose(np.degrees(q),POINT3_JOINTS_DEG)
    def test_failed_recovery_is_persisted_and_never_replayed(self):
        self.recovery.capture()
        with patch.object(self.recovery,'_move',side_effect=RuntimeError('unknown reply')):
            with self.assertRaises(RuntimeError):self.recovery.recover('vision_failed')
        self.assertEqual(self.store.state['grasp_recovery']['status'],'unknown')
        with patch.object(self.recovery,'_move') as move:
            with self.assertRaises((ValueError,AgentError)):self.recovery.recover('vision_failed')
            move.assert_not_called()
    def test_failed_motion_still_stops_if_persisting_failure_raises(self):
        self.recovery.capture();save=self.store.save
        def failed_save():
            if self.store.state['grasp_recovery']['status']=='unknown':raise OSError('disk full')
            save()
        with patch.object(self.store,'save',side_effect=failed_save),patch.object(self.recovery,'_move',side_effect=RuntimeError('lost reply')):
            with self.assertRaises(RuntimeError):self.recovery.recover('vision_failed')
        self.assertFalse(self.c.valid)
        self.assertEqual(self.c.calls[-1],{'type':'arm.stop','arm':'right'})

    def test_route_collision_or_limit_failure_sends_nothing(self):
        self.recovery.capture();q=np.array(self.c.q);q[3]+=.1
        self.c.models['right'].check=lambda *a:dict(clear=False,lower_mm=0.)
        with self.assertRaises(ValueError):self.recovery._move([self.c.q,q.tolist()])
        self.assertEqual(self.c.calls,[])
    def test_completed_recovery_request_is_not_replayed(self):
        self.recovery.capture()
        def move(points):self.c.q=list(points[-1]);return points
        with patch.object(self.recovery,'_move',side_effect=move) as moving:
            first=self.recovery.recover('vision_failed',request_id='flow_1')
            again=self.recovery.recover('vision_failed',request_id='flow_1')
            self.assertEqual(first,again);self.assertEqual(moving.call_count,2)
    def test_invalid_point3_sends_no_motion(self):
        self.recovery.capture();origin=list(self.c.q);routes=[]
        self.c.models['right'].check=lambda a,b,h:dict(clear=bool(b[0]<=origin[0]+.001),lower_mm=100.)
        def move(points):routes.append(points);self.c.q=list(points[-1]);return points
        with patch.object(self.recovery,'_move',side_effect=move):
            with self.assertRaises(ValueError):self.recovery.recover('vision_failed')
        self.assertEqual(routes,[])
    def test_real_recovery_loop_uses_trajectory_receipt_and_never_commands_gripper(self):
        self.recovery.capture();base_snapshot=self.c.snapshot
        def snapshot():self.c.now+=.03;return base_snapshot()
        self.c.snapshot=snapshot
        def start(side,plan,token,preflight):
            self.assertTrue(preflight(self.c.snapshot(),self.c.now))
            self.c.motion_trajectory=plan;self.c.calls.append({'type':'trajectory','plan':plan})
        def poll(token,preflight):
            self.assertTrue(preflight(self.c.snapshot(),self.c.now))
            self.c.q=list(self.c.motion_trajectory['joint_waypoints_rad'][-1])
            self.c.motion_trajectory=None
            return {'status':'reference_complete'}
        self.c.start_trajectory=start;self.c.poll_trajectory=poll
        with patch('agent.grasp_recovery.time.sleep'):
            out=self.recovery.recover('vision_failed')
        self.assertEqual(out['attempt'],1);self.assertEqual(self.c.grip,-3.14)
        self.assertEqual([c['type'] for c in self.c.calls],['trajectory'])
        self.assertEqual(self.store.state['grasp_recovery']['status'],'ready')

    def test_trace_simplification_preserves_turns(self):
        p=np.zeros((10,6));p[:5,0]=np.arange(5)*.01;p[5:,0]=.04;p[5:,1]=np.arange(1,6)*.01
        q=simplify_trace(p.tolist())
        self.assertEqual(len(q),3);np.testing.assert_allclose(q[1],p[4])

class FlowRecoveryTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        self.store=RunStore.create(tmp.name);self.addCleanup(self.store.close)
        self.tools=Tools(self.store);self.tools.vision_mode='plan_f'
        self.recovery=SimpleNamespace(capture=lambda:None,record_approach=lambda r:None,recover=lambda reason,**kw:{'attempt':1,'completed_at':123.})
    def test_first_observation_failure_recovers_with_fresh_id(self):
        base=self.tools.execute;seen=[]
        def execute(name,args,ident):
            seen.append(ident)
            if len(seen)==1:return {'ok':True,'data':{'status':'target_not_found'}}
            return base(name,args,ident)
        self.tools.execute=execute
        flow=BatteryFlow(self.tools,review_plan=review,vision_mode='plan_f',recovery=self.recovery)
        self.assertEqual(flow.run()['status'],'completed')
        self.assertEqual(len(seen),len(set(seen)))
        self.assertNotIn('target_cache',flow.flow)
        self.assertTrue(self.store.state['mission']['point3_refinement'])
        self.assertIn('PICK_AT_PREGRASP',self.store.state['mission']['evidence_barriers'])
    def test_new_recovery_clears_old_plan_and_preview_but_not_call_journal(self):
        flow=BatteryFlow(self.tools,review_plan=review,vision_mode='plan_f',recovery=self.recovery)
        self.store.state['mission'].update(approach_plan={'old':True},plan={'old':True},pick_correction_mm=50)
        self.store.state.update(grasp_preview='old.json',latest_battery_observations={'d405':'old'},
                                battery_observation_attempts={'d405':12})
        self.store.state['calls']['previous']={'proof':True}
        flow.flow.update(phase='recover_pose',recovery_reason='preview_failed',target_cache={'A':'old'})
        self.assertEqual(flow.run()['status'],'completed')
        self.assertNotIn('grasp_preview',self.store.state)
        self.assertIn('previous',self.store.state['calls'])
        self.assertNotIn('target_cache',flow.flow)
    def test_unknown_receipt_never_enters_retry(self):
        self.tools.execute=lambda *args:{'ok':False,'error':{'code':'execution_unknown','message':'lost reply'}}
        with patch.object(self.recovery,'recover') as recover:
            out=BatteryFlow(self.tools,review_plan=review,vision_mode='plan_f',recovery=self.recovery).run()
            self.assertEqual(out['status'],'needs_attention');recover.assert_not_called()

    def test_review_error_restarts_from_a_new_observation(self):
        first=True
        def reviewer(operation,plan):
            nonlocal first
            if first:
                first=False
                raise ValueError('IK unavailable')
            return review(operation,plan)
        with patch.object(self.recovery,'recover',return_value={'completed_at':123.}) as recover:
            out=BatteryFlow(self.tools,review_plan=reviewer,vision_mode='plan_f',recovery=self.recovery).run()
        self.assertEqual(out['status'],'completed');recover.assert_called_once()
        self.assertGreaterEqual(self.tools.calls.count('observe_battery'),3)

    def test_point3_recovery_enters_wrist_refinement_without_initial_approach(self):
        base=self.tools.execute;seen=[]
        def execute(name,args,ident):
            seen.append((name,copy.deepcopy(args)))
            result=base(name,args,ident)
            if name=='plan_battery_pick':
                self.store.state['mission'].update(stage='PICK_PLANNED',plan=result['data'])
                result['data']['refinement']={'status':'aligned'}
            return result
        self.tools.execute=execute
        flow=BatteryFlow(self.tools,review_plan=review,vision_mode='plan_f',recovery=self.recovery)
        flow.flow.update(phase='recover_pose',recovery_reason='vision_failed')
        self.assertEqual(flow.run()['status'],'completed')
        self.assertEqual(seen[0],('observe_battery',{'camera':'d405'}))
        self.assertNotIn('execute_battery_approach',[name for name,args in seen])
        self.assertNotIn('plan_coarse_alignment',[name for name,args in seen])
        self.assertTrue(flow.flow['head'])

    def test_recognition_planning_and_ordinary_errors_restart_the_full_grasp_flow(self):
        for failure in ('scene_changed','pick_plan_failed','observer_search_failed','value_error'):
            with self.subTest(failure=failure):
                self.store.state.pop('battery_flow',None)
                self.store.state['mission']={'stage':'IDLE'}
                self.tools.calls=[];base=Tools.execute.__get__(self.tools);first=True
                def execute(name,args,ident):
                    nonlocal first
                    if first:
                        first=False
                        if failure=='value_error':raise ValueError('invalid perception output')
                        return {'ok':False,'error':{'code':failure,'message':'not localized'}}
                    return base(name,args,ident)
                self.tools.execute=execute
                with patch.object(self.recovery,'recover',return_value={'completed_at':123.}) as recover:
                    out=BatteryFlow(self.tools,review_plan=review,vision_mode='plan_f',recovery=self.recovery).run()
                self.assertEqual(out['status'],'completed');recover.assert_called_once()
                self.assertEqual(self.tools.calls[0],'observe_battery')
                self.assertIn('preview_battery_grasp',self.tools.calls)
                self.assertIn('plan_battery_pick',self.tools.calls)
                self.assertEqual(self.tools.calls[-3:],['verify_hold','stow_for_carry','return_to_carry'])


class FreshRecoveryObservationTests(unittest.TestCase):
    def test_pre_recovery_camera_frame_cannot_be_reused_after_motion(self):
        from agent.battery_tools import BatteryExecutionTools
        from agent.config import Config
        import time
        with tempfile.TemporaryDirectory() as directory:
            store=RunStore.create(directory)
            try:
                now=time.time()
                store.state['battery_task_settings']={'vision_mode':'plan_f'}
                store.state['mission']['evidence_barriers']={'GRASP_RECOVERY':{
                    'completed_at':now,'previous_observation_ids':[]}}
                tools=BatteryExecutionTools(Config(),store)
                old=store.save_observation({'captured_at':now-1,'cameras':{
                    'd405':{'captured_at':now-1,'image_size_px':[640,480]}}},{})['observation_id']
                fresh=store.save_observation({'captured_at':now+1,'cameras':{
                    'd405':{'captured_at':now+1,'image_size_px':[640,480]}}},{})['observation_id']
                with patch.object(tools,'_perception',return_value={'status':'localized','observation_id':old}):
                    r=tools.execute('observe_battery',{'camera':'d405'},'old')
                self.assertEqual(r['error']['code'],'post_action_evidence_required')
                with patch.object(tools,'_perception',return_value={'status':'localized','observation_id':fresh}):
                    r=tools.execute('observe_battery',{'camera':'d405'},'fresh')
                self.assertTrue(r['ok'],r)
            finally:store.close()
