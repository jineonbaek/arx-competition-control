import sys
import tempfile
import time
import unittest
import uuid
from pathlib import Path
from test_control import Controller,FakeAdapter,configured,Rejected
from navigation_session import NavigationSession

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'navigation/polarbear_adapt'))
try:
    from task_client import TaskClient
    from task_handoff import HandoffHub
    from task_http import TaskServer
except ImportError:HandoffHub=None


class Route:
    def __init__(self,hub):
        self.hub=hub;self.sent=[]
        self.definition=dict(route_id=str(uuid.uuid4()),map_id='map',points=[
            dict(type='task',point_id=p,id=p,pose=[1.,0.,0.]) for p in ('point_1','point_2')])
        self.status='待开始 0/2';self.results={};self.online=True
        self.hub.set_route(self.definition,state='draft',running=False,index=0)
    def snapshot(self):
        return dict(route=dict(available=self.online,definition=self.definition,status=self.status,drive_status=''))
    def command(self,action,ident,route_id=None):
        self.sent.append((action,ident));self.results[ident]=dict(status='published')
        self.status='执行中 0/2' if action=='start' else '已取消 0/0'
        if action=='clear':self.hub.cancel()
    def receipt(self,ident):return self.results.get(ident)


@unittest.skipIf(HandoffHub is None,'Navigation counterpart is maintained/deployed separately')
class NavigationSessionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.hub=HandoffHub(Path(self.temp.name)/'journal.json');self.server=TaskServer(self.hub,0)
        self.addCleanup(self.server.close)
        self.ctrl=Controller(configured(),FakeAdapter());self.ctrl.update_body(5.,[0.]*4,0.,0.,0.)
        self.lease=self.ctrl.reserve_competition(0)
        import threading
        self.context=dict(run_id='competition-test',request_id='nav-one',competition_token=self.lease,
                          controller=self.ctrl,cancelled=threading.Event(),token=None)
        self.route=Route(self.hub)
        self.session=NavigationSession(self.ctrl,self.route,TaskClient('http://127.0.0.1:'+str(self.server.port)))
        self.addCleanup(self.session.close)
    def arrive(self,point,mission=None):
        return self.hub.begin(dict(mission_id=mission or str(uuid.uuid4()),point_id=point,
             route_point_id=point,map_id='map',goal_pose=[1.,0.,0.],arrival_pose=[1.,0.,0.],pose_stamp_ns=1))
    def poll(self,stage,expected):
        end=time.monotonic()+2
        while time.monotonic()<end:
            result=stage.inspect(self.context)
            if result['status']==expected:return result
            time.sleep(.02)
        self.fail(str(result))

    def test_real_protocol_arrival_accept_complete_and_cancel_without_manual_gap(self):
        self.assertFalse(self.hub.state()['worker_ready']);self.assertEqual(self.route.sent,[])
        self.ctrl.set_competition_navigation(self.lease,True)
        first=self.session.stage('point_1');first.start(self.context)
        self.assertTrue(self.hub.state()['worker_ready']);self.assertEqual(len(self.route.sent),1)
        visit=self.arrive('point_1');self.poll(first,'completed')
        self.ctrl.set_competition_navigation(self.lease,False)
        token=self.ctrl.reserve_motion(self.ctrl.command_epoch,competition=self.lease)
        self.context['token']=token;self.session.before_work(self.context,'pick')
        self.assertEqual(self.hub.state()['task']['status'],'running')
        with self.assertRaises(Rejected):self.ctrl.submit(dict(type='lift',target=5.),'manual')
        self.ctrl.release_motion(token);self.context['token']=None
        self.ctrl.set_competition_navigation(self.lease,True)
        original_event=self.session.client.event;completion_ids=[]
        def uncertain_reply(*args,**kwargs):
            result=original_event(*args,**kwargs)
            if args[1]=='complete':
                completion_ids.append(kwargs['request_id'])
                if len(completion_ids)==1:raise TimeoutError('response lost after server accepted')
            return result
        self.session.client.event=uncertain_reply
        second=self.session.stage('point_2');second.start(self.context)
        self.assertEqual(self.hub.state()['task']['status'],'resume_pending')
        self.assertEqual(len(completion_ids),2)
        self.assertEqual(completion_ids[0],completion_ids[1])
        self.hub.acknowledge(visit['visit_id'])
        self.arrive('point_2',visit['mission_id']);self.poll(second,'completed')
        self.assertTrue(self.session.stop(self.context)['stopped'])
        self.assertEqual(self.hub.state()['task']['status'],'cancelled')
        self.assertFalse(self.hub.state()['task']['navigation_ready'])
        self.assertEqual([x[0] for x in self.route.sent],['start','clear'])
        self.ctrl.release_competition(self.lease)

    def test_wrong_place_or_mission_never_reports_arrival(self):
        self.ctrl.set_competition_navigation(self.lease,True)
        first=self.session.stage('point_1');first.start(self.context)
        self.arrive('point_2')
        with self.assertRaisesRegex(ValueError,'地点|到访'):self.poll(first,'completed')
        self.session.stop(self.context);self.ctrl.release_competition(self.lease)

    def test_failed_route_and_late_accept_do_not_replay_start(self):
        first=self.session.stage('point_1');first.start(self.context)
        self.route.status='已取消 0/0'
        self.assertEqual(first.inspect(self.context)['status'],'failed')
        self.assertEqual(len(self.route.sent),1)
        self.session.stop(self.context)
        with self.assertRaises(ValueError):first.resume(self.context)
        self.ctrl.release_competition(self.lease)

    def test_offline_or_busy_route_cannot_register_worker_or_send_start(self):
        self.route.online=False
        with self.assertRaises(ValueError):self.session.stage('point_1').start(self.context)
        self.assertFalse(self.hub.state()['worker_ready']);self.assertEqual(self.route.sent,[])
        self.ctrl.release_competition(self.lease)

    def test_failed_navigation_cancel_keeps_exclusion_and_can_retry_stop(self):
        self.session.stage('point_1').start(self.context)
        original=self.route.command
        def reject(action,ident,route_id=None):self.route.results[ident]=dict(status='rejected',reason='cancel unavailable')
        self.route.command=reject
        with self.assertRaisesRegex(ValueError,'cancel unavailable'):self.session.stop(self.context)
        self.assertTrue(self.ctrl.snapshot()['competition_reserved'])
        with self.assertRaises(Rejected):self.ctrl.reserve_motion(self.ctrl.command_epoch)
        self.route.command=original
        self.assertTrue(self.session.stop(self.context)['stopped'])
        self.ctrl.release_competition(self.lease)

    def run_total(self,place=True,stop_in_pick=False):
        import threading
        from competition_task import CompetitionTask
        from test_competition_task import Module
        self.ctrl.release_competition(self.lease)
        for side in ('left','right'):
            self.ctrl.update_arm(side,[0.]*6,-1.,[0.]*6)
        send=self.ctrl.adapter.arm
        def reflect(side,joints,gripper,tcp,mode,**options):
            send(side,joints,gripper,tcp,mode,**options)
            self.ctrl.update_arm(side,list(joints),gripper,list(tcp))
        self.ctrl.adapter.arm=reflect
        trace=[];checked=[]
        modules={key:Module(trace,key) for key in ['pick','carry','place']}
        if not place:modules.pop('place')
        modules['navigate_pick']=self.session.stage('point_1')
        modules['navigate_place']=self.session.stage('point_2')
        def check_work(ctx):
            checked.append((self.hub.state()['task']['point_id'],self.hub.state()['task']['status']))
            with self.assertRaises(Rejected):self.ctrl.submit(dict(type='lift',target=5.),'manual')
        modules['pick'].on_start=check_work
        if stop_in_pick:modules['pick'].status='running'
        task=CompetitionTask(self.ctrl,Path(self.temp.name)/'total',modules=modules,navigation=self.session)
        self.addCleanup(task.close)
        finish=threading.Event();errors=[]
        def virtual_navigation():
            number=0
            while not finish.wait(.01):
                state=self.hub.state();visit=state.get('task')
                if self.route.sent and state['worker_ready'] and number==0:
                    self.arrive('point_1');number=1
                elif visit and visit['status']=='resume_pending':
                    self.assertFalse(self.ctrl.snapshot()['motion_reserved'])
                    with self.assertRaises(Rejected):self.ctrl.submit(dict(type='lift',target=5.),'manual')
                    self.hub.acknowledge(visit['visit_id'])
                    if number==1:self.arrive('point_2',visit['mission_id']);number=2
                    else:self.route.status='全部到达并停稳 2/2'
        def run_navigation():
            try:virtual_navigation()
            except BaseException as error:errors.append(error)
        t=threading.Thread(target=run_navigation,daemon=True);t.start()
        task.action('browser',dict(action='start',request_id='total',expected_epoch=self.ctrl.command_epoch,environment_clear=True))
        end=time.monotonic()+4
        while time.monotonic()<end:
            view=task.snapshot()
            if stop_in_pick and any(x[0]=='point_1' for x in checked):
                task.action('browser',dict(action='stop',request_id='stop'));stop_in_pick=False
            if not task.active['worker'].is_alive():break
            time.sleep(.01)
        finish.set();t.join(1)
        if errors:raise errors[0]
        self.assertFalse(task.active['worker'].is_alive(),str(task.snapshot()))
        return task.snapshot(),trace,checked

    def test_total_sequence_real_handoff_never_opens_manual_between_jobs(self):
        view,trace,checked=self.run_total()
        self.assertEqual(view['status'],'completed',view)
        self.assertEqual([x[1] for x in trace if x[0]=='start'],['pick','carry','place'])
        self.assertEqual(checked,[('point_1','running')])
        self.assertTrue(view['manual_available'])
        self.assertFalse(self.ctrl.snapshot()['competition_reserved'])

    def test_missing_place_cancels_route_and_returns_manual_without_fake_complete(self):
        view,trace,_=self.run_total(place=False)
        self.assertEqual(view['status'],'needs_attention',view)
        self.assertIn('能力未接入',view['reason'])
        self.assertIsNone(self.hub.state()['task'])
        self.assertEqual(self.route.sent,[])
        self.assertTrue(view['manual_available'])

    def test_stop_during_grasp_cancels_navigation_too(self):
        view,trace,_=self.run_total(stop_in_pick=True)
        self.assertEqual(view['status'],'paused',view)
        self.assertEqual([x[1] for x in trace if x[0]=='start'],['pick'])
        self.assertEqual([x[0] for x in self.route.sent],['start','clear'])
        self.assertTrue(view['manual_available'])
