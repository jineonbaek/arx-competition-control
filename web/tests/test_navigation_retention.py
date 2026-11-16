"""Private loopback protocol and inert Controller: never starts grasp or SDK."""
import time,unittest,uuid
import test_navigation_session as f

class SessionRetention(unittest.TestCase):
    setUp=f.NavigationSessionTests.setUp
    arrive=f.NavigationSessionTests.arrive
    poll=f.NavigationSessionTests.poll
    def test_observer_offline_after_start_waits_without_restarting_route(self):
        stage=self.session.stage('point_1');stage.start(self.context);self.route.online=False
        self.assertEqual(stage.inspect(self.context)['status'],'running')
        self.assertEqual(len(self.route.sent),1);self.assertIsNone(self.hub.active)
        self.route.online=True;self.arrive('point_1');self.poll(stage,'completed')
    def test_missing_start_receipt_waits_without_duplicate_start(self):
        stage=self.session.stage('point_1');stage.start(self.context)
        self.route.results.clear();self.session.command_started-=4
        self.assertEqual(stage.inspect(self.context)['status'],'running');self.assertEqual(len(self.route.sent),1)
    def test_channel_outage_longer_than_lease_reconnects_same_instance(self):
        stage=self.session.stage('point_1');stage.start(self.context)
        original=self.session.client._request;until=time.monotonic()+2.4;instance=self.session.client.instance_id
        def interrupted(*a,**kw):
            if time.monotonic()<until:raise TimeoutError('fixture channel offline')
            return original(*a,**kw)
        self.session.client._request=interrupted
        time.sleep(2.1);self.assertEqual(stage.inspect(self.context)['status'],'running')
        end=time.monotonic()+3
        while time.monotonic()<end:
            if self.hub.state()['worker_ready'] and not self.session.error:break
            time.sleep(.05)
        self.assertTrue(self.hub.state()['worker_ready']);self.assertFalse(self.session.error)
        self.assertEqual(self.session.client.instance_id,instance);self.assertEqual(len(self.route.sent),1)
    def test_uncertain_visit_keeps_waiting_without_fake_complete(self):
        stage=self.session.stage('point_1');stage.start(self.context);visit=self.arrive('point_1')
        self.session.client.event(visit,'fail',request_id=str(uuid.uuid4()),error='worker_or_task_timeout')
        time.sleep(.25)
        self.assertEqual(stage.inspect(self.context)['status'],'running')
        self.assertEqual(self.hub.active['status'],'needs_attention');self.assertEqual(len(self.route.sent),1)

if __name__=='__main__':unittest.main()
