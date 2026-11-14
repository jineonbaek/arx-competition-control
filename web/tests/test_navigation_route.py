import json
import tempfile
import unittest
import subprocess
import sys
from pathlib import Path
from navigation_route import RouteMailbox, RouteRelay


class RouteRelayTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.sent=[]
        self.view=dict(route=dict(available=True,definition=dict(route_id='route'),status='待开始 0/2'))
        self.box=RouteMailbox(self.root,lambda:self.view)
        self.relay=RouteRelay(self.root)
        self.relay.available=lambda:True
        self.relay.definition=dict(route_id='route')
        self.relay.publish=lambda action:self.sent.append(action)

    def test_route_commands_once_and_changed_route_rejected(self):
        self.box.command('start','one','route');self.relay.tick();self.relay.tick()
        self.assertEqual(self.sent,['start'])
        self.assertEqual(self.box.receipt('one')['status'],'published')
        self.box.command('start','changed','other');self.relay.tick()
        self.assertEqual(self.box.receipt('changed')['status'],'rejected')
        self.assertEqual(self.sent,['start'])
        self.box.command('clear','cancel');self.relay.tick()
        self.assertEqual(self.sent,['start','clear'])

    def test_observer_module_imports_without_web_agent_pythonpath(self):
        source=str(Path(__file__).resolve().parents[1]/'src')
        subprocess.run([sys.executable,'-I','-c',
            'import sys;sys.path.insert(0,'+repr(source)+');import navigation_route'],check=True)

    def test_restart_does_not_replay_pending_start_and_stale_observer_cannot_queue_motion(self):
        self.box.command('start','pending','route')
        restarted=RouteRelay(self.root);restarted.available=lambda:True
        restarted.publish=lambda action:self.sent.append(action)
        restarted.tick();self.assertEqual(self.sent,[])
        self.view.clear()
        with self.assertRaises(ValueError):self.box.command('start','offline','route')
        self.assertEqual(json.loads((self.root/'navigation-command.json').read_text())['request_id'],'pending')
