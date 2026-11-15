"""Online upgrade diagnostics with fake transport; no ROS/device startup."""
import unittest
from unittest.mock import patch
from test_control import Controller,FakeAdapter,configured
from agent.cabinet_readiness import readiness,admission_probe


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.now=100.;self.adapter=FakeAdapter();self.c=Controller(configured(),self.adapter,lambda:self.now)
        self.c.wrist_reason=lambda side:''
        for side in ('left','right'):self.c.update_arm(side,[0.]*6,-1.,[0.]*6)
        self.c.update_body(5.,[0.]*4,0.,0.,0.)
        self.adapter.trajectory_status=lambda side:dict(driver_instance='fixture',command_epoch=7,
                                                       request_id='',status='idle',enabled=True,ready=True)

    def test_status_describes_code_installation_without_physical_permission(self):
        r=readiness(self.c)
        self.assertTrue(r['bridge_installed']);self.assertTrue(r['driver_binding_supported'])
        self.assertFalse(r['execution_enabled']);self.assertFalse(r['physical_provider_configured'])
        self.assertEqual(self.adapter.calls,[]);self.assertIsNone(self.c.motion_owner)

    def test_fixed_probe_exercises_live_controller_admission_without_dispatch(self):
        before=self.c.command_epoch;r=admission_probe(self.c,{'op':'admission_probe'})
        self.assertTrue(r['blocked_before_reservation']);self.assertFalse(r['motion_commands_sent'])
        self.assertFalse(r['execution_enabled']);self.assertIn('physical provider',r['reason'])
        self.assertEqual(before,self.c.command_epoch);self.assertIsNone(self.c.motion_owner)
        self.assertEqual(self.adapter.calls,[])

    def test_probe_refuses_targets_enable_overrides_and_unknown_operations(self):
        for data in ({},{'op':'start'},{'op':'admission_probe','target':[1]*6},
                     {'op':'admission_probe','enable':True},None):
            with self.assertRaises(ValueError):admission_probe(self.c,data)
        self.assertEqual(self.adapter.calls,[])

    def test_probe_never_runs_during_existing_motion_or_competition(self):
        token=self.c.reserve_motion(self.c.command_epoch)
        with self.assertRaises(ValueError):admission_probe(self.c,{'op':'admission_probe'})
        self.assertIs(self.c.motion_owner,token);self.assertEqual(self.adapter.calls,[])

    def test_older_controller_reports_required_upgrade_without_traffic(self):
        self.c.start_trajectory=lambda side,plan,token,preflight:None
        r=readiness(self.c);self.assertFalse(r['driver_binding_supported'])
        p=admission_probe(self.c,{'op':'admission_probe'})
        self.assertTrue(p['blocked_before_reservation']);self.assertIn('driver-epoch',p['reason'])
        self.assertEqual(self.adapter.calls,[])

    def test_missing_feedback_is_explicit_rejection_not_server_error(self):
        c=Controller(configured(),self.adapter)
        r=admission_probe(c,{'op':'admission_probe'})
        self.assertTrue(r['blocked_before_reservation']);self.assertIn('feedback',r['reason'])
        self.assertEqual(self.adapter.calls,[]);self.assertIsNone(c.motion_owner)

    def test_competition_and_chassis_ownership_prevent_probe(self):
        snapshot=self.c.snapshot
        for key in ('competition_reserved','chassis_active'):
            self.c.snapshot=lambda:dict(snapshot(),**{key:True})
            with self.assertRaises(ValueError):admission_probe(self.c,{'op':'admission_probe'})
        self.assertEqual(self.adapter.calls,[])

    def test_bridge_probe_has_no_caller_supplied_approval_or_provider(self):
        with patch('agent.cabinet_readiness.ControllerBridge') as bridge:
            bridge.side_effect=ValueError('fixture error')
            with self.assertRaises(ValueError):admission_probe(self.c,{'op':'admission_probe'})
            args,kw=bridge.call_args
            self.assertNotIn('physical_provider',kw)


if __name__=='__main__':unittest.main()
