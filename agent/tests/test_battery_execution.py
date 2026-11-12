import unittest

import numpy as np

from agent.battery_execution import prepare_step
from agent.tests.test_execution_policy import policy
from web.src.handeye_column import ColumnModel


class Controller:
    def __init__(self, joints):
        self.joints = joints
        self.models = {'right': ColumnModel()}

    def motion_valid(self, token):
        return token == 'owned'

    def snapshot(self):
        return {
            'arms': {
                'left': {'joints': [0.] * 6},
                'right': {'joints': self.joints.tolist()},
            },
            'body': {'height': 5.},
        }

    def clock(self):
        return 1.


class BatteryExecutionTests(unittest.TestCase):
    def test_fixed_observer_uses_actual_wrist_path_length(self):
        start = np.array([
            -0.00057220458984375, -0.00782012939453125, 0.01277923583984375,
            -0.00095367431640625, -0.00247955322265625, -0.00782012939453125,
        ])
        end = np.array([
            -0.0005017597831963648, 0.42296212128643385, 0.697700527519362,
            -0.22935973127059087, -0.0022447758099674504, -0.005134052911537012,
        ])
        settings = policy()
        coarse = dict(settings['coarse'], timing_limits=settings['timing_limits'])
        plan = {
            'arm': 'right', 'executable': True, 'collision_checked': True,
            'collision_free': True, 'environment_collision_checked': True,
            'ik_checked': True,
            'initial_joints_rad': {'left': [0.] * 6, 'right': start.tolist()},
            'lift': {'fixed': True, 'current_rad': 0.46, 'target_rad': 5.52,
                     'acceptance_rad': 5.},
            'observer': {'status': 'fixed_observer_candidate', 'joints_rad': end.tolist()},
        }
        step = prepare_step('execute_coarse_alignment', plan, Controller(start), 'owned',
                            policies={'coarse': coarse}, pick_setup=None,
                            preflight=lambda *args: True)

        self.assertLess(step.plan['time_from_start_s'][-1], 60.)


if __name__ == '__main__':
    unittest.main()
