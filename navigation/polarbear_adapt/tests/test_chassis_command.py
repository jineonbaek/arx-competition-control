import io
import json
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from chassis_command import WebChassis, follow_velocity


class Commands(unittest.TestCase):
    def test_axes_and_combined_speed_limit(self):
        for x,y,yaw in [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(1,-1,.7),(0,0,-.7)]:
            vx,vy,w = follow_velocity(x,y,yaw)
            self.assertLessEqual(math.hypot(vx,vy),.08000001)
            self.assertAlmostEqual(vx*y,vy*x)
            self.assertGreaterEqual(vx*x+vy*y,0)
            self.assertGreaterEqual(w*yaw,0)
            self.assertLessEqual(abs(w),.18)

    @patch('chassis_command.build_opener')
    def test_console_requests(self, opener):
        transport=opener.return_value.open
        transport.side_effect=[io.BytesIO(b''),io.BytesIO(b'{"ok":true}'),io.BytesIO(b'{"ok":true}'),io.BytesIO(b'{"ok":false}')]
        bridge=WebChassis()
        bridge.send((.04,-.03,.1))
        bridge.send((0.,0.,0.))
        with self.assertRaises(RuntimeError):
            bridge.send((.01,0.,0.))
        requests=[entry.args[0] for entry in transport.call_args_list[1:]]
        self.assertEqual(json.loads(requests[0].data),dict(type='chassis',x=.04,y=-.03,omega=.1,sequence=1))
        self.assertEqual(requests[0].get_header('Origin'),bridge.origin)
        self.assertEqual(json.loads(requests[1].data),dict(type='chassis.stop',sequence=2))


if __name__=='__main__':
    unittest.main()
