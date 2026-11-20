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
        self.assertTrue(all(c['type']=='chassis.end_output' for c,_ in self.commands))
        self.step.cancel()
        self.assertEqual(self.commands[-1][0]['type'],'chassis.end_output')
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
        self.assertTrue(self.step.poll().get('waiting_for_pose'))
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

    def test_retreat_ignores_blocked_costmaps_but_validates_pose_source(self):
        reader=_PoseReader.__new__(_PoseReader)
        blocked=Mock();blocked.along.return_value={'first':{'occupied':True}}
        blocked.inspect.return_value={'first_cell':{'occupied':True}}
        node=Mock();node.grids={'map':(None,blocked),'odom':(None,blocked)}
        node.checked.side_effect=lambda **kw: None if kw.get('occupancy') is False else (_ for _ in ()).throw(ValueError('map_footprint_blocked'))
        reader.node=node
        reader.check_motion(np.array([-.1,0.,0.]),50.)
        node.checked.assert_called_once_with(occupancy=False)
        node.validate_motion_proof.assert_called_once_with(50.)
        blocked.along.assert_not_called();blocked.inspect.assert_not_called()

    def test_retreat_still_rejects_invalid_pose_at_send_boundary(self):
        reader=_PoseReader.__new__(_PoseReader);reader.node=Mock()
        reader.node.validate_motion_proof.side_effect=ValueError('motion_pose_changed_during_proof')
        with self.assertRaisesRegex(ValueError,'motion_pose_changed_during_proof'):
            reader.check_motion(np.array([-.1,0.,0.]),50.)

    def test_pose_read_skips_occupancy_check(self):
        reader=_PoseReader.__new__(_PoseReader)
        reader.node=Mock();reader.node.received_pose=True;reader.node.target=self.odom.copy()
        reader.node.motion_poses.return_value=(self.odom,self.odom,50.)
        reader.executor=Mock();reader.previous_pose=None
        self.assertEqual(reader.read()[2],50.)
        reader.node.checked.assert_called_once_with(occupancy=False)

    def test_only_costmap_staleness_is_excluded_from_retreat(self):
        from agent.chassis_retreat import _retreat_stale_inputs
        follow=SimpleNamespace(Follow=SimpleNamespace(stale_inputs=lambda node:
            ['map_costmap','/pb/scan','odom_costmap','/localization/odometry']))
        with patch.dict('sys.modules',{'follow_preview':follow}):
            self.assertEqual(_retreat_stale_inputs(object()),['/pb/scan','/localization/odometry'])


if __name__=='__main__':unittest.main()
