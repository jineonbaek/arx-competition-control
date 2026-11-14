"""Competition endpoints retain existing session/Origin and strict JSON checks."""
import json
from unittest.mock import Mock

import test_battery_task_http as battery_http
from server import make_handler
from http.server import ThreadingHTTPServer
from test_control import Controller, configured, FakeAdapter
import threading
import unittest


class CompetitionHTTPTests(unittest.TestCase):
    request = staticmethod(battery_http.BatteryTaskHTTPTests.request)
    session = battery_http.BatteryTaskHTTPTests.session
    def server(self, task):
        adapter=FakeAdapter()
        server=ThreadingHTTPServer(('127.0.0.1',0),
            make_handler(Controller(configured(),adapter),competition_task=task))
        server.daemon_threads=True;server.stopping=threading.Event()
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(thread.join);self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown);self.addCleanup(server.stopping.set)
        return server,adapter

    def test_mission_routes_and_map_share_state_without_robot_writes(self):
        task=Mock();task.snapshot.return_value=dict(status='idle',tasks=[],command_epoch=0)
        task.action.return_value=dict(status='running',run_id='same-run')
        server,adapter=self.server(task)
        self.assertEqual(self.request(server,'GET','/api/competition-task')[0],403)
        headers=self.session(server)
        code,_,body=self.request(server,'GET','/api/navigation-view',headers=headers)
        self.assertEqual(code,200)
        self.assertEqual(json.loads(body)['competition']['status'],'idle')
        for action in ('start','stop','continue'):
            data=dict(action=action,request_id=action)
            code,_,_=self.request(server,'POST','/api/competition-task',json.dumps(data),headers)
            self.assertEqual(code,202)
        self.assertEqual(task.action.call_count,3)
        self.assertEqual(self.request(server,'POST','/api/competition-task','{}',dict(headers,Origin='http://other'))[0],403)
        self.assertEqual(self.request(server,'POST','/api/competition-task','{"action":"start","action":"stop"}',headers)[0],400)
        self.assertEqual(task.action.call_count,3)
        self.assertEqual(adapter.calls,[])

    def test_missing_backend_is_explicit(self):
        server,adapter=self.server(None);headers=self.session(server)
        self.assertEqual(self.request(server,'GET','/api/competition-task',headers=headers)[0],503)
        self.assertEqual(self.request(server,'POST','/api/competition-task','{}',headers)[0],503)
        self.assertEqual(adapter.calls,[])
