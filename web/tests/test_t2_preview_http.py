import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import unittest

from competition_task import CompetitionTask
from server import make_handler
from test_control import Controller, FakeAdapter, configured


class PreviewStatus:
    def snapshot(self):
        return dict(status='idle', enabled=True, mode='recorded_preview', preview_only=True)


class T2PreviewHTTPTests(unittest.TestCase):
    def test_preview_mode_is_visible_but_endpoint_cannot_start_motion(self):
        with tempfile.TemporaryDirectory() as folder:
            adapter = FakeAdapter()
            controller = Controller(configured(), adapter)
            task = CompetitionTask(controller, Path(folder))
            server = ThreadingHTTPServer(('127.0.0.1', 0),
                make_handler(controller, competition_task=task, t2_preview=PreviewStatus()))
            server.daemon_threads = True
            server.stopping = threading.Event()
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            try:
                conn = http.client.HTTPConnection('127.0.0.1', server.server_port)
                headers = {'Cookie':'arx_web_session=50c70394-9559-4694-b9c7-b14385be22ce'}
                conn.request('GET', '/api/t2-preview', headers=headers)
                response = conn.getresponse(); state = json.loads(response.read())
                self.assertEqual(response.status, 200)
                self.assertEqual(state['mode'], 'recorded_preview')
                conn.request('GET', '/api/competition-task', headers=headers)
                response = conn.getresponse(); state = json.loads(response.read())
                self.assertEqual(response.status, 200)
                place = next(p for p in state['tasks'] if p['id']=='place')
                self.assertIn('预览', place['label'])
                self.assertIn('不执行放置', place['label'])
                self.assertTrue(state['t2_preview']['preview_only'])
                conn.request('POST', '/api/t2-preview', '{}', headers)
                response = conn.getresponse(); response.read()
                self.assertEqual(response.status, 404)
                self.assertEqual(adapter.calls, [])
                self.assertIsNone(task.active)
                conn.close()
            finally:
                server.stopping.set(); server.shutdown(); server.server_close()
                worker.join(2.)
                task.close()
