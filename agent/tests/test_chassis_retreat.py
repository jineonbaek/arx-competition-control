"""Recorded poses and inert transport; never starts ROS or hardware."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from agent.chassis_retreat import ChassisRetreatStep,_PoseReader


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

    def test_stops_after_200mm_without_forward_correction_then_uses_existing_stop_window(self):
        with patch('urllib.request.build_opener') as http:
            self.step.start();http.assert_not_called()
        self.assertEqual(self.commands,[])
        self.assertEqual(self.step.poll()['status'],'backing_chassis')
        np.testing.assert_allclose(self.step.target,[2.,2.8,np.pi/2])
        command,token=self.commands[0]
        self.reader.check_motion.assert_called_once()
        self.assertEqual(token,'owned');self.assertEqual(command['type'],'chassis.nav_sdk')
        self.assertLess(command['x'],0.);self.assertAlmostEqual(command['y'],0.)
        self.assertEqual(command['omega'],0.)
        self.odom[1]=2.79;self.now+=.1
        self.assertEqual(self.step.poll()['status'],'backing_chassis')
        self.assertEqual(self.commands[-1][0]['type'],'chassis.end_output')
        self.assertFalse(self.step.done)
        for _ in range(12):
            self.now+=.1
            receipt=self.step.poll()
        self.assertEqual(receipt['status'],'chassis_stopped')
        self.reader.close.assert_called_once()
        self.assertEqual(len(self.commands),2)
        self.assertFalse(any(c['type']=='chassis.nav_sdk' and c['x']>0 for c,_ in self.commands))

    def test_no_motion_until_first_pose_and_cancel_uses_existing_stop(self):
        self.reader.read.side_effect=lambda:None
        self.step.start()
        self.assertEqual(self.step.poll()['status'],'backing_chassis')
        self.assertEqual(self.commands,[])
        self.step.cancel()
        self.assertEqual(self.commands[0][0]['type'],'chassis.end_output')
        self.reader.close.assert_called_once()

    def test_over_400mm_fails_after_stopping_without_forward_correction(self):
        self.step.start();self.step.poll()
        self.odom[1]=2.59;self.now+=.1
        self.assertEqual(self.step.poll()['status'],'backing_chassis')
        with self.assertRaisesRegex(ValueError,'chassis_retreat_out_of_range'):
            for _ in range(12):
                self.now+=.1
                self.step.poll()
        self.assertFalse(any(c['type']=='chassis.nav_sdk' and c['x']>0 for c,_ in self.commands))

    def test_existing_pose_failure_is_not_hidden_and_cancel_stops(self):
        self.step.start();self.step.poll()
        self.reader.read.side_effect=ValueError('motion_pose_stale_or_invalid')
        with self.assertRaisesRegex(ValueError,'motion_pose_stale_or_invalid'):self.step.poll()
        self.step.cancel()
        self.assertEqual(self.commands[-1][0]['type'],'chassis.end_output')
        self.reader.close.assert_called_once()

    def test_real_controller_keeps_task_owner_and_arm_lift_while_using_native_transport(self):
        from web.tests.test_control import Controller,FakeAdapter,configured
        adapter=FakeAdapter();adapter.chassis_output=Mock()
        config=configured();config['chassis_backend']='memory'
        controller=Controller(config,adapter,lambda:self.now)
        for side in ('left','right'):controller.update_arm(side,[0.]*6,-.9,[0.]*6)
        controller.update_body(5.1,[0.]*4,0.,0.,0.)
        token=controller.reserve_motion(controller.command_epoch)
        step=ChassisRetreatStep(controller,token)
        step.start();step.poll()
        adapter.chassis_output.assert_called_once()
        self.assertTrue(controller.snapshot()['chassis_active'])
        step.cancel()
        snapshot=controller.snapshot()
        self.assertFalse(snapshot['chassis_active'])
        self.assertTrue(controller.motion_valid(token))
        self.assertEqual(snapshot['arms']['right']['gripper'],-.9)
        self.assertEqual(snapshot['body']['height'],5.1)
        self.assertEqual(adapter.chassis_output.call_count,2)
        self.assertEqual(adapter.chassis_output.call_args.args[0]['sdk'],[0.,0.,0.])
        self.assertEqual(adapter.calls,[])

    def test_existing_map_and_odom_sweeps_and_source_validation_are_reused(self):
        reader=_PoseReader.__new__(_PoseReader);reader.measured=np.zeros(3)
        clear=Mock();clear.along.return_value={'first':False}
        node=Mock();node.motion_poses.return_value=(self.odom,self.odom,50.)
        node.grids={'map':(None,clear),'odom':(None,clear)};reader.node=node
        reader.check_motion(np.array([-.1,0.,0.]),50.)
        node.checked.assert_called_once();node.validate_motion_proof.assert_called_once_with(50.)
        self.assertTrue(clear.along.called)

    def test_existing_obstacle_rejection_prevents_send_boundary_validation(self):
        reader=_PoseReader.__new__(_PoseReader);reader.measured=np.zeros(3)
        blocked=Mock();blocked.along.return_value={'first':{'occupied':True}}
        node=Mock();node.motion_poses.return_value=(self.odom,self.odom,50.)
        node.grids={'map':(None,blocked),'odom':(None,blocked)};reader.node=node
        with self.assertRaisesRegex(ValueError,'map_sweep_blocked'):
            reader.check_motion(np.array([-.1,0.,0.]),50.)
        node.validate_motion_proof.assert_not_called()


if __name__=='__main__':unittest.main()
