"""Exercise scope binding through the real execution adapter without transport."""
import copy
import unittest
import numpy as np
from web.tests import test_battery_step_preparation as fixtures
from agent.pick_execution import ApproachTrajectoryStep

class ScopedExecutionTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.PreparationTests();self.f.setUp()

    def test_approach_uses_exact_single_checked_segment(self):
        p=self.f.plan
        p['motion_scope']='approach'
        p['approach']['end_rad']=p['joint_waypoints_rad'][-1][:]
        step=self.f.prepare().approach
        self.assertIsInstance(step,ApproachTrajectoryStep)
        np.testing.assert_array_equal(step.plan['joint_waypoints_rad'],p['joint_waypoints_rad'])
        self.assertEqual(self.f.f.adapter.calls,[])

    def test_scope_cannot_be_changed_by_requesting_another_operation(self):
        for scope,operation in [('approach','execute_pick'),('correction','execute_pick'),
                                ('geometry','execute_pick'),('insertion','execute_battery_approach')]:
            with self.subTest(scope=scope,operation=operation):
                self.f.plan['motion_scope']=scope
                with self.assertRaisesRegex(ValueError,'scope'):self.f.prepare(operation)
        self.assertEqual(self.f.f.adapter.calls,[])

    def test_tampered_approach_endpoint_is_rejected_without_transport(self):
        self.f.plan['motion_scope']='approach'
        with self.assertRaisesRegex(ValueError,'bound'):self.f.prepare()
        self.assertEqual(self.f.f.adapter.calls,[])

if __name__=='__main__':unittest.main()
