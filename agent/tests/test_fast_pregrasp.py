"""Fast taught pregrasp routing, with no hardware transport."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from agent.fast_pick import FAST_PREGRASP_JOINTS_DEG,prepare_pregrasp


class FastPregraspTests(unittest.TestCase):
    def setUp(self):
        from agent.tests.test_grasp_pose_recovery import RecoveryTests
        fixture=RecoveryTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        self.fixture=fixture;self.store=fixture.store
        self.store.state['battery_flow']={'phase':'head'}
        def next_phase(phase):
            self.store.state['battery_flow']['phase']=phase;self.store.save()
        self.flow=SimpleNamespace(recovery=fixture.recovery,store=self.store,_next=next_phase)

    def test_logged_target_enters_refinement_without_changing_lift_or_gripper(self):
        target=np.radians(FAST_PREGRASP_JOINTS_DEG).tolist()
        grip=self.fixture.c.grip;lift=self.fixture.c.lift
        with patch.object(self.fixture.recovery,'_move',return_value=[self.fixture.c.q,target]) as move:
            prepare_pregrasp(self.flow)
        np.testing.assert_allclose(np.degrees(move.call_args.args[0][-1]),
            [13.092,43.766,42.166,6.003,14.637,-87.707])
        self.assertEqual(self.store.state['battery_flow']['phase'],'wrist')
        self.assertTrue(self.store.state['mission']['point3_refinement'])
        self.assertIn('PICK_AT_PREGRASP',self.store.state['mission']['evidence_barriers'])
        self.assertNotIn('approach_plan',self.store.state['mission'])
        self.assertEqual(self.fixture.c.grip,grip);self.assertEqual(self.fixture.c.lift,lift)
        self.assertEqual(self.fixture.c.calls,[])
        self.assertEqual(self.fixture.recovery.state['outbound_trace'][-1],target)

    def test_unknown_move_stops_and_does_not_enter_refinement(self):
        with patch.object(self.fixture.recovery,'_move',side_effect=ValueError('lost receipt')):
            with self.assertRaisesRegex(ValueError,'lost receipt'):prepare_pregrasp(self.flow)
        self.assertFalse(self.fixture.c.valid)
        self.assertTrue(self.fixture.c.stop_pending)
        self.assertEqual(self.fixture.c.calls,[{'type':'arm.stop','arm':'right'}])
        self.assertEqual(self.store.state['battery_flow']['phase'],'head')
        self.assertNotIn('fast_pregrasp',self.store.state)
