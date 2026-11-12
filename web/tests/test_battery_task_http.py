"""HTTP contract for the fixed battery task; fake task only."""
import http.client
import json
from http.server import ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from control import Controller
from server import make_handler
from test_control import FakeAdapter,configured


class BatteryTaskHTTPTests(unittest.TestCase):
    def server(self,task):
        adapter=FakeAdapter()
        server=ThreadingHTTPServer(("127.0.0.1",0),
            make_handler(Controller(configured(),adapter),battery_task=task))
        server.daemon_threads=True;server.stopping=threading.Event()
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(thread.join)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.addCleanup(server.stopping.set)
        return server,adapter

    @staticmethod
    def request(server,method,path,body=None,headers=None):
        host=f"127.0.0.1:{server.server_port}"
        conn=http.client.HTTPConnection(host,timeout=3)
        conn.request(method,path,body,headers or {})
        response=conn.getresponse()
        result=response.status,dict(response.getheaders()),response.read()
        conn.close()
        return result

    def session(self,server):
        status,headers,_=self.request(server,"GET","/")
        self.assertEqual(status,200)
        return {"Cookie":headers["Set-Cookie"].split(";")[0],
                "Origin":f"http://127.0.0.1:{server.server_port}",
                "Content-Type":"application/json"}

    def test_session_origin_strict_json_and_task_forwarding(self):
        task=Mock()
        task.snapshot.return_value={"status":"idle","available":True,"command_epoch":4}
        task.action.return_value={"status":"running","available":False,"run_id":"run-1"}
        server,adapter=self.server(task)
        self.assertEqual(self.request(server,"GET","/api/battery-task")[0],403)
        good=self.session(server)
        status,_,body=self.request(server,"GET","/api/battery-task",headers=good)
        self.assertEqual(status,200);self.assertEqual(json.loads(body)["command_epoch"],4)
        task.snapshot.assert_called_once()
        self.assertEqual(self.request(server,"POST","/api/battery-task",
            '{"action":"start"}',{**good,"Origin":"http://other"})[0],403)
        self.assertEqual(self.request(server,"POST","/api/battery-task",
            '{"action":"start","action":"cancel"}',good)[0],400)
        task.action.assert_not_called()
        status,_,body=self.request(server,"POST","/api/battery-task",
            '{"action":"start","request_id":"r1","expected_epoch":4}',good)
        self.assertEqual(status,202);self.assertEqual(json.loads(body)["run_id"],"run-1")
        task.action.assert_called_once()
        self.assertEqual(adapter.calls,[])

    def test_unconfigured_task_is_explicitly_unavailable(self):
        server,_=self.server(None);good=self.session(server)
        self.assertEqual(self.request(server,"GET","/api/battery-task",headers=good)[0],503)
        self.assertEqual(self.request(server,"POST","/api/battery-task",
            '{"action":"start","request_id":"r1","expected_epoch":0}',good)[0],503)


if __name__=="__main__":unittest.main()
