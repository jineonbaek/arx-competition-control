"""Real scheduler/proxy protocol with mocked pose and inert chassis output."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from agent.chassis_retreat import ChassisRetreatStep
from agent.tests.test_retreat_ros_handoff import ColdTransport
from arx_chassis.proxy import ChassisProxy
from navigation.polarbear_adapt.chassis_command import WebChassis


class RetreatSessionIdentityTests(unittest.TestCase):
    def test_wait_before_first_move_uses_navigation_session_through_stop(self):
        transport = ColdTransport()
        with patch('arx_chassis.proxy.shared_transport', return_value=transport):
            proxy = ChassisProxy()
        now = [50.]
        odom = np.array([2., 3., np.pi / 2])
        reader = Mock()
        reader.now.side_effect = lambda: now[0]
        reader.read.return_value = None
        tokens = []

        def submit(command, token):
            tokens.append(token)
            sdk = None if command['type'] == 'chassis.end_output' else tuple(
                command[k] for k in ('x', 'y', 'omega'))
            return proxy.submit(command, token, sdk)

        controller = SimpleNamespace(config={'chassis_backend': 'ros'}, chassis=proxy,
                                     clock=lambda: now[0], submit=submit)
        step = ChassisRetreatStep(controller, 'owned')
        with patch('agent.chassis_retreat._PoseReader', return_value=reader), patch(
                'agent.chassis_retreat.WebChassis',
                side_effect=lambda **kw: WebChassis(wall=lambda: transport.wall, **kw)):
            step.start()
            self.assertTrue(step.poll()['waiting_for_pose'])
            self.assertEqual(transport.outputs, [])
            reader.read.side_effect = lambda: (odom.copy(), odom.copy(), now[0])
            now[0] += .1
            step.poll()
            self.assertLess(transport.outputs[-1]['sdk'][0], 0.)
            odom[1] -= .21
            for _ in range(14):
                now[0] += .1
                step.poll()
            self.assertTrue(step.done)
            step.cancel()

        self.assertEqual(set(proxy.sessions), {('owned', 'navigation')})
        self.assertEqual(len({m['source_id'] for m in transport.messages}), 1)
        self.assertEqual(set(tokens), {'owned'})
        self.assertEqual(transport.messages[0]['type'], 'chassis.end_output')
        self.assertEqual(transport.messages[-1]['type'], 'chassis.end_output')
        self.assertEqual(transport.outputs[-1]['sdk'], [0., 0., 0.])
        reader.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
