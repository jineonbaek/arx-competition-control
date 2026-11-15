"""Recorded poses and inert transport; never starts ROS or hardware."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from agent.chassis_retreat import ChassisRetreatStep


class RetreatTests(unittest.TestCase):
    def setUp(self):
        self.now=50.;self.odom=np.array([2.,3.,np.pi/2]);self.commands=[]
        self.reader=Mock()
        self.reader.now.side_effect=lambda:self.now
        self.reader.read.side_effect=lambda:(np.array([99.,-99.,1.]),self.odom.copy(),self.now)
        patcher=patch('agent.chassis_retreat._PoseReader',return_value=self.reader)
        patcher.start();self.addCleanup(patcher.stop)
        self.controller=SimpleNamespace(clock=lambda:self.now,
            submit=lambda command,token:self.commands.append((command,token)) or {'ok':True})
        self.step=ChassisRetreatStep(self.controller,'owned')

    def test_reuses_navigation_transport_and_odom_heading_then_existing_stop_window(self):
        with patch('navigation.polarbear_adapt.chassis_command.build_opener') as http:
            self.step.start();http.assert_not_called()
        self.assertEqual(self.commands,[])
        self.assertEqual(self.step.poll()['status'],'backing_chassis')
        np.testing.assert_allclose(self.step.target,[2.,2.9,np.pi/2])
        command,token=self.commands[0]
        self.assertEqual(token,'owned');self.assertEqual(command['type'],'chassis.nav_sdk')
        self.assertLess(command['x'],0.);self.assertAlmostEqual(command['y'],0.)
        self.assertEqual(command['omega'],0.)
        self.odom=self.step.target.copy();self.now+=.1
        self.assertEqual(self.step.poll()['status'],'backing_chassis')
        self.assertEqual(self.commands[-1][0]['type'],'chassis.stop')
        self.assertFalse(self.step.done)
        for _ in range(12):
            self.now+=.1
            receipt=self.step.poll()
        self.assertEqual(receipt['status'],'chassis_stopped')
        self.reader.close.assert_called_once()
        self.assertEqual(len(self.commands),2)

    def test_no_motion_until_first_pose_and_cancel_uses_existing_stop(self):
        self.reader.read.side_effect=lambda:None
        self.step.start()
        self.assertEqual(self.step.poll()['status'],'backing_chassis')
        self.assertEqual(self.commands,[])
        self.step.cancel()
        self.assertEqual(self.commands[0][0],{'type':'chassis.stop','sequence':1})
        self.reader.close.assert_called_once()

    def test_existing_pose_failure_is_not_hidden_and_cancel_stops(self):
        self.step.start();self.step.poll()
        self.reader.read.side_effect=ValueError('motion_pose_stale_or_invalid')
        with self.assertRaisesRegex(ValueError,'motion_pose_stale_or_invalid'):self.step.poll()
        self.step.cancel()
        self.assertEqual(self.commands[-1][0]['type'],'chassis.stop')
        self.reader.close.assert_called_once()

    def test_real_controller_keeps_task_owner_and_arm_lift_while_using_native_transport(self):
        from web.tests.test_control import Controller,FakeAdapter,configured
        adapter=FakeAdapter();adapter.nav_chassis_sdk=Mock()
        controller=Controller(configured(),adapter,lambda:self.now)
        for side in ('left','right'):controller.update_arm(side,[0.]*6,-.9,[0.]*6)
        controller.update_body(5.1,[0.]*4,0.,0.,0.)
        token=controller.reserve_motion(controller.command_epoch)
        step=ChassisRetreatStep(controller,token)
        step.start();step.poll()
        adapter.nav_chassis_sdk.assert_called_once()
        self.assertTrue(controller.snapshot()['chassis_active'])
        step.cancel()
        snapshot=controller.snapshot()
        self.assertFalse(snapshot['chassis_active'])
        self.assertTrue(controller.motion_valid(token))
        self.assertEqual(snapshot['arms']['right']['gripper'],-.9)
        self.assertEqual(snapshot['body']['height'],5.1)
        self.assertTrue(all(kind=='chassis' for kind,args in adapter.calls))


if __name__=='__main__':unittest.main()
