"""Contact feedback and state transitions, without any live command transport."""
import time
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

from insertion_contact import ContactMonitor, InsertionContact


class LinearModel:
    def wrist_transform(self, q, height):
        result = np.eye(4)
        result[0, 3] = q[0] * .05
        return result


def feedback(t, progress, reference, current=1., status='running'):
    q = [progress / 50., 0., 0., 0., 0., 0.]
    ref = [reference / 50., 0., 0., 0., 0., 0.]
    arm = dict(joints=q, received=t, fresh=True)
    if current is not None:
        arm['currents'] = [current] * 6
    return dict(arms={'right': arm}, body={'height': 0.}), dict(
        joint_reference_rad=ref, status=status)


class ContactMonitorTests(unittest.TestCase):
    def setUp(self):
        s, d = feedback(0., 0., 0.)
        self.monitor = ContactMonitor(LinearModel(), np.eye(4), np.eye(4), [1., 0., 0.], s, d)

    def sample(self, t, progress, reference, current=1., status='running'):
        s, d = feedback(t, progress, reference, current, status)
        return self.monitor.sample(s, d, t)

    def learn(self):
        for i in range(1, 13):
            self.sample(i * .05, float(i), float(i))

    def test_clear_continuous_travel_and_quintic_stationary_start_do_not_trip(self):
        for i in range(1, 61):
            self.sample(i * .05, 0., 0., 3.)
        for i in range(61, 101):
            self.sample(i * .05, (i-60)*.1, (i-60)*.1, 3.)

    def test_sustained_current_and_tracking_growth_stop_before_two_seconds(self):
        self.learn()
        with self.assertRaises(InsertionContact) as stopped:
            for i in range(13, 33):
                self.sample(i * .05, 12., 14., 3.)
        self.assertEqual(stopped.exception.evidence['kind'], 'current_contact')
        self.assertLess(stopped.exception.evidence['sample_time'], 1.7)
        self.assertFalse(stopped.exception.evidence['stop_confirmed'])

    def test_static_payload_current_and_one_current_spike_do_not_fake_fast_contact(self):
        self.learn()
        for i in range(13, 33):
            self.sample(i * .05, 12., 14., 4. if i == 20 else 1.)

    def test_missing_current_keeps_the_position_stall_fallback(self):
        self.learn()
        with self.assertRaises(InsertionContact) as stopped:
            for i in range(13, 60):
                self.sample(i * .05, 12., 14., None)
        self.assertEqual(stopped.exception.evidence['kind'], 'progress_stall')
        self.assertGreaterEqual(stopped.exception.evidence['sample_time'], 2.6)

    def test_reference_completion_does_not_hide_an_unreached_position(self):
        self.learn()
        with self.assertRaises(InsertionContact):
            for i in range(13, 40):
                self.sample(i * .05, 12., 14., 3., 'reference_complete')

    def test_load_supporting_reference_offset_is_removed_from_contact_error(self):
        s, d = feedback(0., 0., 3., 4.)
        self.monitor = ContactMonitor(LinearModel(), np.eye(4), np.eye(4), [1.,0.,0.], s,d)
        for i in range(1, 70):
            self.sample(i * .05, 0., 3., 4.)

    def test_stale_and_repeated_samples_never_prove_contact(self):
        self.learn()
        s, d = feedback(.65, 12., 14., 4.)
        for now in [.65, .7, .75]:
            self.monitor.sample(s, d, now)
        with self.assertRaises(ValueError) as failure:
            self.monitor.sample(s, d, 2.)
        self.assertNotIsInstance(failure.exception, InsertionContact)


class CurrentFeedbackTests(unittest.TestCase):
    def test_optional_current_is_copied_and_invalid_or_missing_current_is_cleared(self):
        from control import Controller
        from test_control import configured, FakeAdapter
        c = configured(); c.pop('chassis_backend', None)
        ctrl = Controller(c, FakeAdapter(), lambda: 10.)
        values = [1., 2., 3., 4., 5., 6.]
        ctrl.update_arm('right', [0.]*6, -.6, [0.]*6, currents=values)
        values[0] = 99.
        self.assertEqual(ctrl.arms['right']['currents'], [1.,2.,3.,4.,5.,6.])
        for invalid in (None, [float('nan')]*6, [1.]*5):
            ctrl.update_arm('right', [0.]*6, -.6, [0.]*6, currents=invalid)
            self.assertIsNone(ctrl.arms['right']['currents'])


class ContactSeatingTests(unittest.TestCase):
    def run_contact(self, travel, depth, *, confirmed=True, stale=False, outside=40.):
        from mouth_push import run_fixed_stroke
        tool=np.eye(4);moves=[]
        def move(phase, length, rotation):
            moves.append(length);tool[0,3] = travel/1000.
            error=InsertionContact(dict(kind='current_contact',stop_confirmed=confirmed,
                reference_reset=confirmed,progress_mm=travel,sample_time=1.,stop_confirmed_at=time.time()))
            error.evidence.update(stopped_travel_mm=travel)
            raise error
        def verify(obs, guard):
            return dict(insertion_depth_verified=depth==150., verified_depth_mm=depth,
                handle_outside_mm=outside, verification_frame_ids=['fresh'],
                verification_image_time=time.time()-(10. if stale else 0.), verification_reason='depth')
        result=run_fixed_stroke(np.eye(4),lambda:tool.copy(),move,lambda:True,verify)
        return result,moves,tool

    def test_near_seat_contact_can_release_only_after_confirmed_stop_and_fresh_depth(self):
        from mouth_push import require_seated_release
        result,moves,tool=self.run_contact(149.,150.)
        self.assertEqual(moves,[150.]);self.assertTrue(result['release_ready'])
        require_seated_release(np.eye(4),tool,np.array([1.,0.,0.]),result)
        tool[0,3]=.145
        with self.assertRaises(ValueError):
            require_seated_release(np.eye(4),tool,np.array([1.,0.,0.]),result)

    def test_early_jam_and_unverified_seat_never_release_or_replay(self):
        for travel,depth,options in [(80.,150.,{}),(149.,140.,{}),(149.,150.,{'stale':True}),
                                     (149.,150.,{'outside':-1.})]:
            result,moves,_=self.run_contact(travel,depth,**options)
            self.assertFalse(result['release_ready']);self.assertEqual(moves,[150.])
            self.assertTrue(result['contact_detected'])

    def test_contact_after_compensation_outside_seat_zone_cannot_release(self):
        from mouth_push import run_fixed_stroke
        tool=np.eye(4);moves=[]
        def move(phase,length,rotation):
            moves.append(length);tool[0,3]+=length/1000.
            if len(moves)==3:
                raise InsertionContact(dict(kind='current_contact',stop_confirmed=True,reference_reset=True,
                    stopped_travel_mm=170.,stop_confirmed_at=time.time()))
        def verify(obs,guard):
            return dict(insertion_depth_verified=len(moves)==3,verified_depth_mm=[120.,130.,150.][len(moves)-1],
                handle_outside_mm=40.,verification_frame_ids=[str(len(moves))],verification_image_time=time.time())
        result=run_fixed_stroke(np.eye(4),lambda:tool.copy(),move,lambda:True,verify)
        self.assertEqual(moves,[150.,10.,10.]);self.assertFalse(result['release_ready'])

    def test_unconfirmed_stop_and_ordinary_faults_never_reach_verification(self):
        with self.assertRaises(ValueError):self.run_contact(149.,150.,confirmed=False)
        from mouth_push import run_fixed_stroke
        def fail(*args):raise ValueError('driver replaced')
        with self.assertRaisesRegex(ValueError,'driver replaced'):
            run_fixed_stroke(np.eye(4),lambda:np.eye(4),fail,lambda:True,
                             lambda *a:self.fail('fault treated as contact'))


class ReachedPathTests(unittest.TestCase):
    def test_partial_contact_return_uses_no_unreached_planned_nodes(self):
        from fast_visual_level import StrokeTrace
        trace=StrokeTrace([0.]*6)
        self.assertEqual(trace.stopped([.01]*6), [[.01]*6])
        trace.sample(10., [.03]*6)
        self.assertEqual(trace.stopped([.04]*6), [[.03]*6,[.04]*6])


class ContactStopTests(unittest.TestCase):
    def make_motion(self, failure=None):
        from docking_motion import DockingMotion
        class Controller:
            def __init__(self):
                self.now=10.;self.valid=True;self.sent=[]
                self.driver=dict(driver_instance='right',command_epoch=7,status='running',
                    request_id='stroke',reason='',reference_from_trajectory=True,ready=True,enabled=True,joint_reference_rad=[.1]*6)
                self.adapter=SimpleNamespace(trajectory_status=lambda side:dict(self.driver),pending_arm_stops={})
            def clock(self):return self.now
            def sleep(self,t):self.now+=t
            def motion_valid(self,token):return self.valid
            def begin_contact_stop(self,token):self.stop_pending=True
            def complete_contact_stop(self,token):self.stop_pending=False
            def snapshot(self):
                return dict(arms={'right':dict(joints=[.1]*6,received=self.now,fresh=True)},body={'height':0.})
            def submit(self,command,token):
                self.sent.append(command)
                self.adapter.pending_arm_stops['right']=SimpleNamespace(done=lambda:failure!='pending',
                    result=lambda:SimpleNamespace(success=failure!='rejected'))
                if failure!='still_running':
                    self.driver.update(command_epoch=self.driver['command_epoch']+1,status='failed',
                        reason='manual_stop',reference_from_trajectory=False)
                if failure=='fault_during_stop':self.driver['reason']='lease_expired'
                if failure=='replaced':self.driver['driver_instance']='replacement'
                if failure=='cancelled':self.valid=False
        ctrl=Controller()
        motion=DockingMotion(ctrl,'token',guard=lambda s,t:True,sleep=ctrl.sleep)
        motion.active='arm'
        motion.arm_handle=dict(driver_instance='right',command_epoch=6,request_id='stroke')
        return ctrl,motion

    def test_confirmed_stop_preserves_clamp_and_records_fresh_stable_feedback(self):
        ctrl,motion=self.make_motion()
        contact=InsertionContact(dict(kind='current_contact',stop_confirmed=False))
        state=motion.confirm_contact_stop(contact)
        self.assertEqual(ctrl.sent,[{'type':'arm.stop','arm':'right'}])
        self.assertTrue(contact.evidence['stop_confirmed'])
        self.assertTrue(contact.evidence['reference_reset'])
        self.assertGreaterEqual(ctrl.now,10.25)
        self.assertGreater(state['arms']['right']['received'],10.)
        self.assertTrue(ctrl.valid)
        self.assertFalse(motion.contact_stop_pending)

    def test_uncertain_stop_driver_change_and_cancel_never_confirm_contact(self):
        for failure in ('pending','rejected','still_running','replaced','cancelled','fault_during_stop'):
            with self.subTest(failure=failure):
                ctrl,motion=self.make_motion(failure)
                contact=InsertionContact(dict(kind='current_contact',stop_confirmed=False))
                with self.assertRaises(ValueError):motion.confirm_contact_stop(contact)
                self.assertFalse(contact.evidence['stop_confirmed'])
                self.assertTrue(motion.contact_stop_pending)

    def test_fault_or_changed_trajectory_before_stop_is_never_seating_contact(self):
        for change in (dict(status='failed',reason='tracking_error',reference_from_trajectory=False),
                       dict(command_epoch=8),dict(request_id='another'),dict(driver_instance='new')):
            ctrl,motion=self.make_motion();ctrl.driver.update(change)
            contact=InsertionContact(dict(stop_confirmed=False))
            with self.assertRaises(ValueError):motion.confirm_contact_stop(contact)
            self.assertFalse(contact.evidence['stop_confirmed'])
            self.assertTrue(motion.contact_stop_pending)

    def test_stationary_feedback_cannot_finish_while_contact_decision_is_pending(self):
        ctrl,motion=self.make_motion()
        motion._rest('arm',10.,lambda:None,settled_when=lambda s:ctrl.now>=10.8)
        self.assertGreaterEqual(ctrl.now,10.8)

    def test_motion_executor_converts_only_contact_to_a_confirmed_stop(self):
        ctrl,motion=self.make_motion()
        ctrl.models={'right':SimpleNamespace(check=lambda *a:dict(clear=True))}
        ctrl.config={'arms':{'right':{'joint_limits_deg':[[-180.,180.]]*6}}}
        ctrl.started=False
        def start(side, plan, token, **kwargs):
            self.assertTrue(kwargs['preflight'](ctrl.snapshot(),ctrl.now))
            ctrl.started=True;ctrl.driver['command_epoch']+=1
            return dict(driver_instance='right',command_epoch=7,request_id='stroke')
        ctrl.start_trajectory=start
        ctrl.poll_trajectory=lambda *a,**kw:self.fail('Contact must stop before next trajectory heartbeat')
        def observe():
            if ctrl.started:
                raise InsertionContact(dict(kind='current_contact',stop_confirmed=False))
        plan=dict(joint_waypoints_rad=[[.1]*6,[.11]*6],time_from_start_s=[0.,1.],interpolation='segment_quintic_scalar')
        with self.assertRaises(InsertionContact) as caught:
            motion.arm([plan],observe,expected_start=dict(joints=[.1]*6,height=0.),
                       expected_driver=dict(driver_instance='right',command_epoch=7))
        self.assertTrue(caught.exception.evidence['stop_confirmed'])
        self.assertEqual(ctrl.sent,[dict(type='arm.stop',arm='right')])
        self.assertTrue(ctrl.valid)


class StopOwnershipTests(unittest.TestCase):
    def test_competition_owner_cannot_release_or_move_through_an_uncertain_contact_stop(self):
        from control import Controller,Rejected
        from test_control import configured,FakeAdapter
        ctrl=Controller(configured(),FakeAdapter(),lambda:10.)
        for side in ('right','left'):ctrl.update_arm(side,[0.]*6,-.5,[0.]*6)
        ctrl.update_body(0.,[0.]*4,0.,0.,0.)
        competition=ctrl.reserve_competition(ctrl.command_epoch)
        token=ctrl.reserve_motion(ctrl.command_epoch,competition=competition)
        ctrl.begin_contact_stop(token)
        self.assertTrue(ctrl.snapshot()['contact_stop_pending'])
        with self.assertRaises(Rejected):ctrl.release_motion(token)
        with self.assertRaises(Rejected):ctrl.release_competition(competition)
        with self.assertRaises(Rejected):ctrl.submit(dict(type='gripper',arm='right',position=-1.),token)
        ctrl.submit(dict(type='arm.stop',arm='right'),token)
        with self.assertRaises(Rejected):ctrl.release_motion(token)
        with self.assertRaises(Rejected):ctrl.complete_contact_stop(object())
        ctrl.complete_contact_stop(token)
        self.assertFalse(ctrl.snapshot()['contact_stop_pending'])
        ctrl.release_motion(token);ctrl.release_competition(competition)
        self.assertFalse(ctrl.snapshot()['motion_reserved'])
        self.assertFalse(ctrl.snapshot()['competition_reserved'])

    def test_automatic_placement_retains_owner_when_contact_stop_is_uncertain(self):
        from fast_visual_level import FastVisualLevel
        for pending in (True,False):
            calls=[];saved={}
            ctrl=SimpleNamespace(snapshot=lambda:dict(command_epoch=1),reserve_motion=lambda e:'owner',
                                 release_motion=lambda t:calls.append(t))
            task=FastVisualLevel(ctrl,None);task.auto_insert_160=True
            task.motion=SimpleNamespace(contact_stop_pending=pending,hold=lambda:None)
            task.save=lambda **values:saved.update(values)
            with patch('fast_visual_level.wait_entry_feedback',side_effect=ValueError('injected stop failure')):
                task._run_task()
            self.assertEqual(calls,[] if pending else ['owner'])
            if pending:self.assertTrue(saved['control_retained'])

    def test_current_axis_contact_release_return_and_uncertain_stop_cleanup(self):
        from current_axis_push import CurrentAxisPush
        root=Path(__file__).resolve().parents[2]
        for travel,pending in ((149.,False),(80.,False),(149.,True)):
            moves=[];commands=[]
            class Model:
                reach=.7
                def wrist_transform(self,q,h):
                    tool=np.eye(4);tool[0,3]=q[0];return tool
            class Ctrl:
                def __init__(self):
                    self.config=json.loads((root/'web/config/control.json').read_text())
                    self.models={'right':Model()};self.adapter=self;self.q=[0.]*6;self.grip=-.5
                    self.target=0.;self.token=None
                def trajectory_status(self,side):
                    return dict(driver_instance='right',command_epoch=1,status='idle',ready=True,enabled=True,
                        joint_reference_rad=self.q,gripper_target_raw=self.target)
                def snapshot(self):
                    return dict(command_epoch=1,motion_reserved=self.token is not None,
                        competition_reserved=False,chassis_active=False,body=dict(height=0.,fresh=True),
                        arms={s:dict(joints=list(self.q) if s=='right' else [0.]*6,gripper=self.grip,
                                     fresh=True,received=time.monotonic()) for s in ('left','right')})
                def reserve_motion(self,e):self.token=object();return self.token
                def release_motion(self,t):self.token=None
                def motion_valid(self,t):return self.token is t
                def submit(self,command,token,**kwargs):
                    kwargs['preflight'](self.snapshot(),None)
                    commands.append(command);self.target=self.grip=command['position'];return dict(ok=True)
            class Motion:
                handoffs=[]
                def __init__(self,ctrl,*a,**kw):self.ctrl=ctrl;self.contact_stop_pending=False
                def hold(self):pass
                def settled_arm(self,guard):guard();return self.ctrl.snapshot()
                def arm(self,parts,guard,**kwargs):
                    moves.append(parts)
                    if len(moves)==1:
                        self.ctrl.q[0]=travel/1000.;guard()
                        self.contact_stop_pending=pending
                        if pending:raise ValueError('stop acknowledgement missing')
                        raise InsertionContact(dict(kind='current_contact',stop_confirmed=True,
                            reference_reset=True,stop_confirmed_at=time.time()))
                    self.ctrl.q=list(parts[-1]['joint_waypoints_rad'][-1]);guard();return self.ctrl.snapshot()
            def plan(*args,**kwargs):
                first=list(args[1]);end=first.copy();end[0]+=(args[4][0]-args[3][0])/1000.
                return dict(joint_waypoints_rad=[first,end],timing=dict(time_from_start_s=[0.,1.]))
            with tempfile.TemporaryDirectory() as tmp:
                folder=Path(tmp);(folder/'config').mkdir()
                (folder/'config/right-grip-tcp.json').write_text(json.dumps(dict(T_wrist_from_tool=np.eye(4).tolist())))
                (folder/'config/battery-execution.json').write_bytes((root/'config/battery-execution.json').read_bytes())
                ctrl=Ctrl();task=CurrentAxisPush(ctrl,folder)
                def verify(obs,guard):
                    self.assertFalse(pending,'Unconfirmed stop reached D405 verification')
                    return dict(insertion_depth_verified=True,verified_depth_mm=150.,handle_outside_mm=40.,
                        verification_image_time=time.time(),verification_frame_ids=['post-stop'])
                task.check_fixed_depth=verify
                with patch('current_axis_push.plan_insert',side_effect=plan),patch('current_axis_push.DockingMotion',Motion):
                    task.action(dict(op='start',request_id='contact',expected_epoch=1,mouth_then_push=True))
                    task.thread.join(4.)
                self.assertFalse(task.thread.is_alive())
                result=task.snapshot()
                if pending:
                    self.assertTrue(result['control_retained']);self.assertIsNotNone(ctrl.token)
                    self.assertFalse(result['released']);self.assertEqual(commands,[])
                elif travel<140.:
                    self.assertEqual(result['status'],'held');self.assertEqual(result['next_phase'],'realign_required')
                    self.assertEqual(commands,[]);self.assertEqual(len(moves),1);self.assertIsNone(ctrl.token)
                else:
                    self.assertEqual(result['status'],'completed_verified',result.get('reason'))
                    self.assertTrue(result['released']);self.assertTrue(result['returned']);self.assertEqual(len(commands),1)
                    self.assertEqual(commands[0]['type'],'gripper');self.assertIsNone(ctrl.token)
                    reverse=[q[0] for part in moves[1] for q in part['joint_waypoints_rad']]
                    self.assertLessEqual(max(reverse),.149);self.assertAlmostEqual(ctrl.q[0],0.)


if __name__ == '__main__':unittest.main()
