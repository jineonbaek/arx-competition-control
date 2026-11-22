"""Final review regressions, fake transports only."""
import unittest,time,copy
import numpy as np
from unittest.mock import patch
from agent.tests import test_grasp_pose_recovery as f
from agent.tests import test_grasp_relaxed_arrival as a
from agent.grasp_recovery import GraspRecovery
from agent.battery_flow import BatteryFlow
from agent.tests.test_battery_flow import Tools,review

class RecoveryFixTests(unittest.TestCase):
    setUp=f.RecoveryTests.setUp
    def test_duplicate_approach_receipt_does_not_add_another_roundtrip(self):
        self.recovery.capture();via=list(self.c.q);via[1]+=.2;end=via.copy();end[0]+=.2
        data={'execution_id':'approach1','return_trace_rad':[list(self.c.q),via,end]}
        self.recovery.record_approach(data);before=copy.deepcopy(self.recovery.state['outbound_trace'])
        self.recovery.record_approach(data)
        self.assertEqual(self.recovery.state['outbound_trace'],before)
    def test_failed_stop_retains_real_controller_owner_after_reference_complete(self):
        from control import Controller,Rejected
        from test_control import configured,FakeAdapter
        c=Controller(configured(),FakeAdapter(),lambda:10.)
        for side in ('left','right'):c.update_arm(side,[0.]*6,-3.14,[0.]*6)
        c.update_body(12.,[0.]*4,0,0,0);c.wrist_reason=lambda side:''
        competition=c.reserve_competition(c.command_epoch)
        token=c.reserve_motion(c.command_epoch,competition=competition)
        self.b.controller=c;self.b.token=token
        recovery=GraspRecovery(self.b,environment_clear=True);recovery.capture()
        with patch.object(recovery,'_move',side_effect=ValueError('not settled')),patch.object(c,'submit',side_effect=Rejected('stop failed')):
            with self.assertRaises(ValueError):recovery.recover('vision_failed')
        with self.assertRaises(Rejected):c.release_motion(token)
        with self.assertRaises(Rejected):c.release_competition(competition)
        self.assertTrue(c.snapshot()['motion_reserved'])
    def test_repeated_correction_without_progress_does_not_add_recovery_gate(self):
        tools=Tools(self.store);tools.vision_mode='plan_f';tools.corrections=3
        from types import SimpleNamespace
        r=SimpleNamespace(capture=lambda:None,record_approach=lambda data:True,
                          recover=lambda reason,**kw:{'completed_at':time.time()})
        with patch.object(r,'record_approach',side_effect=[True,False,True,True,True,True]),patch.object(r,'recover',wraps=r.recover) as recovering:
            out=BatteryFlow(tools,review_plan=review,vision_mode='plan_f',recovery=r).run()
        self.assertEqual(out['status'],'completed');self.assertEqual(recovering.call_count,0)
