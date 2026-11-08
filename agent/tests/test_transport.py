import json
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

from agent.config import Config
from agent.contracts import AgentError
from agent.storage import RunStore


class Server:
    def __init__(self, respond):
        self.requests = []
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                self.handle_request()

            def do_POST(self):
                self.handle_request()

            def handle_request(self):
                body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
                owner.requests.append((self.command, self.path, body))
                status, content_type, response = respond(self.path, body)
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.end_headers()
                self.wfile.write(response)

        self.http = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)

    def __enter__(self):
        self.thread.start()
        self.url = 'http://127.0.0.1:' + str(self.http.server_port)
        return self

    def __exit__(self, *args):
        self.http.shutdown()
        self.http.server_close()
        self.thread.join()


def response(value):
    return 200, 'application/json', json.dumps(value).encode()


class TransportTests(unittest.TestCase):
    def test_capture_freezes_source_frame_without_claiming_depth_or_sync(self):
        from agent.observations import ConsoleObservations
        # Minimal JPEG marker fixture; dimensions are tested without a decoder dependency.
        jpeg = bytes.fromhex('ffd8 ffc0 000b 08 000c 0010 01 01 11 00 ffd9')

        def respond(path, body):
            if path.endswith('.jpg'):
                return 200, 'image/jpeg', jpeg
            if path == '/api/state':
                return response({'arms': {}, 'body': {}, 'private_field': 'exclude'})
            return response({'state': 'online', 'sequence': 7, 'captured_at': time.time(),
                             'raw_image_file': 'd435-r-abcd-7.jpg', 'image_file': 'd435-v-abcd-7.jpg',
                             'battery': {'intrinsics': {'fx': 100}}})

        with tempfile.TemporaryDirectory() as root, Server(respond) as server:
            store = RunStore.create(root)
            try:
                capture = ConsoleObservations(Config(console_url=server.url), store).capture(['d435'])
                frame = capture['cameras']['d435']
                self.assertEqual(frame['image_size_px'], [16, 12])
                self.assertEqual(frame['source_image_file'], 'd435-r-abcd-7.jpg')
                self.assertFalse(frame['geometry_ready'])
                self.assertFalse(capture['cross_camera_synchronized'])
                self.assertNotIn('private_field', capture['robot_state'])
                self.assertEqual(store.image_path(capture['observation_id'], 'd435').read_bytes(), jpeg)
                self.assertTrue(all(method == 'GET' for method, _, _ in server.requests))
            finally:
                store.close()

    def test_stale_and_traversal_frames_never_download_image(self):
        from agent.observations import ConsoleObservations
        for metadata in [
            {'state': 'online', 'captured_at': time.time() - 30, 'sequence': 1, 'image_file': 'd405-v-ab-1.jpg'},
            {'state': 'online', 'captured_at': time.time(), 'sequence': 1, 'image_file': '../secrets.jpg'},
        ]:
            with self.subTest(metadata=metadata), tempfile.TemporaryDirectory() as root, Server(lambda p, b: response(metadata)) as server:
                store = RunStore.create(root)
                try:
                    with self.assertRaises(AgentError):
                        ConsoleObservations(Config(console_url=server.url), store).capture(['d405'])
                    self.assertFalse(any(path.endswith('.jpg') for _, path, _ in server.requests))
                finally:
                    store.close()

    def test_stream_assembles_tool_arguments_and_keeps_reasoning_for_protocol(self):
        from agent.model import QwenClient
        pieces = [
            {'delta': {'role': 'assistant', 'reasoning_content': 'protocol context'}},
            {'delta': {'tool_calls': [{'index': 0, 'id': 'call_1', 'type': 'function', 'function': {'name': 'get_system_status', 'arguments': '{'}}]}},
            {'delta': {'tool_calls': [{'index': 0, 'function': {'arguments': '}'}}]}, 'finish_reason': 'tool_calls'},
        ]
        wire = ''.join('data: ' + json.dumps({'choices': [p]}) + '\n\n' for p in pieces) + 'data: [DONE]\n\n'
        with Server(lambda p, b: (200, 'text/event-stream', wire.encode())) as server, patch.dict('os.environ', {'QWEN_BASE_URL': server.url, 'DASHSCOPE_API_KEY': 'fixture-key'}):
            tools = [{'type': 'function', 'function': {'name': 'get_system_status', 'parameters': {'type': 'object'}}}]
            message = QwenClient(Config()).complete([{'role': 'user', 'content': 'test'}], tools)
            payload = json.loads(server.requests[0][2])
            self.assertTrue(payload['parallel_tool_calls'])
            self.assertEqual(payload['max_tokens'], 16384)
            self.assertEqual(message['tool_calls'][0]['function']['arguments'], '{}')
            self.assertEqual(message['reasoning_content'], 'protocol context')
            self.assertEqual(server.requests[0][1], '/chat/completions')

    def test_partial_stream_is_not_a_complete_model_reply(self):
        from agent.model import QwenClient
        wire = 'data: ' + json.dumps({'choices': [{'delta': {'content': 'done'}}]}) + '\n\n'
        with Server(lambda p, b: (200, 'text/event-stream', wire.encode())) as server, patch.dict('os.environ', {'QWEN_BASE_URL': server.url, 'DASHSCOPE_API_KEY': 'fixture-key'}):
            with self.assertRaises(AgentError) as caught:
                QwenClient(Config()).complete([], [])
            self.assertEqual(caught.exception.code, 'model_protocol_error')

    def test_nonstream_and_missing_credentials(self):
        from agent.model import QwenClient
        with patch.dict('os.environ', {}, clear=True):
            with self.assertRaises(AgentError):
                QwenClient(Config()).complete([], [])
        with Server(lambda p, b: response({'choices': [{'message': {'role': 'assistant', 'content': 'ready'}, 'finish_reason': 'stop'}]})) as server:
            cfg = Config(profiles={'local': {'model': 'fixture', 'base_url': server.url, 'stream': False}})
            self.assertEqual(QwenClient(cfg, 'local').complete([], [])['content'], 'ready')


    def test_single_visual_report_policy(self):
        from agent.model import QwenClient
        call = {'id': 'visual_report', 'type': 'function',
                'function': {'name': 'report_visual_result', 'arguments': '{"status":"not_found"}'}}
        body = {'choices': [{'message': {'role': 'assistant', 'tool_calls': [call]}, 'finish_reason': 'tool_calls'}]}
        with Server(lambda p, b: response(body)) as server:
            cfg = Config(profiles={'cloud': {'model': 'fixture', 'base_url': server.url, 'stream': False}})
            reply = QwenClient(cfg).complete([], [{'type': 'function', 'function': {'name': 'report_visual_result'}}],
                                                     parallel_tool_calls=False)
            payload = json.loads(server.requests[0][2])
            self.assertEqual(payload['tool_choice'], 'auto')
            self.assertFalse(payload['parallel_tool_calls'])
            self.assertEqual(reply['tool_calls'][0]['function']['name'], 'report_visual_result')


if __name__ == '__main__':
    unittest.main()
