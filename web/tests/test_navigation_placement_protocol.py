import sys
import tempfile
import threading
import unittest
import uuid
from pathlib import Path

import test_navigation_placement as fixtures
from navigation_session import NavigationSession

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'navigation/polarbear_adapt'))
from task_client import TaskClient
from task_handoff import HandoffHub
from task_http import TaskServer


class NavigationPlacementHTTPTests(fixtures.NavigationPlacementTests):
    def setUp(self):
        super().setUp()
        self.hub=HandoffHub(self.root/'protocol-journal.json')
        self.http=TaskServer(self.hub,0);self.addCleanup(self.http.close)
        client=TaskClient('http://127.0.0.1:'+str(self.http.port));client.register(['point_2'])
        session=NavigationSession(self.ctrl,None,client);session.active=True
        self.context['run_id']='competition-test'
        session.context=self.context
        session.visit=self.hub.begin(dict(mission_id=str(uuid.uuid4()),point_id='point_2',
            route_point_id='cabinet',map_id='room',goal_pose=[0.,0.,0.],arrival_pose=[0.,0.,0.],pose_stamp_ns=1))
        session.mission=session.visit['mission_id']
        session.before_work(self.context,'place')
        self.nav=session;self.module.navigation=session

    # Parent pure-adapter cases are covered in their own class.
    def test_http_complete_is_sent_only_after_real_adapter_success(self):
        self.assertEqual(self.hub.state()['task']['status'],'running')
        self.module.start(self.context);result=self.wait()
        self.assertEqual(result['status'],'completed')
        self.assertEqual(self.hub.state()['task']['status'],'running')
        self.nav.complete_visit(self.context)
        task=self.hub.state()['task']
        self.assertEqual(task['status'],'resume_pending')
        self.assertTrue(task['control_released'])
        self.assertTrue(task['navigation_ready'])
        self.assertTrue(self.hub.blocking())
        self.hub.acknowledge(task['visit_id'])
        self.assertFalse(self.hub.blocking())

    def test_http_failed_reference_cannot_emit_completion(self):
        self.record(False);self.module.start(self.context);result=self.wait()
        self.assertEqual(result['status'],'failed')
        self.assertEqual(self.hub.state()['task']['status'],'needs_attention')
        self.assertFalse(self.hub.state()['task']['navigation_ready'])


# Do not re-run parent tests with a different navigation implementation.
for name in list(fixtures.NavigationPlacementTests.__dict__):
    if name.startswith('test_'):setattr(NavigationPlacementHTTPTests,name,None)

if __name__=='__main__':unittest.main()
