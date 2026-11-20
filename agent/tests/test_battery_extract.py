"""Chassis-only extraction and transport return; all commands are inert."""
import unittest
import json
import tempfile
import time
from types import SimpleNamespace
from pathlib import Path
from web.src.handeye_column import ColumnModel
from agent.grasp_path import TimedJointPath
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
from agent.battery_extract import WristExtractionStep,CarryReturnStep


class ExtractionTests(unittest.TestCase):
    def setUp(self):
        self.pose=np.eye(4);self.pose[:3,3]=[.45,-.15,.70]
        self.pose[:3,:3]=Rotation.from_euler('xyz',[-85,1,2],degrees=True).as_matrix()
        self.commands=[];self.owned=True
        self.retreat=Mock()
        self.retreat.poll.return_value={'status':'chassis_stopped'}
        patched=patch('agent.battery_extract.ChassisRetreatStep',return_value=self.retreat)
        patched.start();self.addCleanup(patched.stop)
        self.now=10.
        self.joints=np.radians([10.,20.,30.,5.,6.,-80.])
        self.arm_policy=dict(tolerance=[.035]*6,max_speed=[.05]*6,
            settle_s=.25,timeout_s=20.,stall_s=2.,max_age_s=.3)
        self.timing=json.loads((Path(__file__).resolve().parents[2]/'config/battery-execution.json').read_text())['timing_limits']
        self.tcp=np.eye(4);self.tcp[0,3]=.148
        self.trajectory_status='reference_complete'
        self.controller=SimpleNamespace(
            clock=lambda:self.now,
            motion_valid=lambda token:self.owned,
            models={'right':ColumnModel()},
            snapshot=lambda:dict(arms={'right':{'joints':self.joints.tolist(),'received':self.now,
                'gripper':-1.,'fresh':True}},body={'height':5.1,'fresh':True}),
            interrupt_motion=lambda token:setattr(self,'owned',False),
            submit=lambda command,token,**kwargs:self.commands.append(command) or {'ok':True},
            log=Mock(),
            start_trajectory=lambda side,plan,token,**kwargs:self.commands.append(dict(type='arm.trajectory',arm=side,**plan)),
            poll_trajectory=lambda token,**kwargs:{'status':self.trajectory_status})
        self.extract_step=WristExtractionStep(self.controller,'owned')
        self.step=CarryReturnStep(self.controller,'owned',
            arm_policy=self.arm_policy,timing_limits=self.timing,wrist_from_tool=self.tcp)

    def finish_return(self):
        self.joints=np.radians([.5,-.5,90.2,-70.2,.1,-89.])
        for _ in range(5):
            self.now+=.1
            result=self.step.poll()
        return result

    def test_chassis_starts_without_wrist_motion_or_ik_and_stops_before_return(self):
        self.assertEqual(self.commands,[])
        self.controller.models['right'].inverse_wrist=Mock(side_effect=AssertionError('No wrist IK'))
        self.assertEqual(self.extract_step.start()['status'],'backing_chassis')
        self.retreat.start.assert_called_once()
        self.assertEqual(self.commands,[])
        self.retreat.poll.return_value={'status':'backing_chassis'}
        self.assertEqual(self.extract_step.poll()['status'],'backing_chassis')
        self.assertEqual(self.commands,[])
        self.retreat.poll.return_value={'status':'chassis_stopped'}
        self.assertEqual(self.extract_step.poll()['status'],'chassis_stopped')
        self.assertEqual(self.commands,[])
        with self.assertRaises(ValueError):self.extract_step.start()
        self.assertEqual(self.step.start()['status'],'returning')
        self.assertEqual(self.commands[0]['type'],'arm.trajectory')
        np.testing.assert_allclose(self.commands[0]['joint_waypoints_rad'][-1],np.radians([0.,0.,90.,-70.,0.,-90.]))
        self.assertEqual(self.finish_return()['status'],'carry_ready')
        self.assertEqual(self.step.poll()['status'],'carry_ready')
        self.assertEqual([c['type'] for c in self.commands],['arm.trajectory'])
        with self.assertRaises(ValueError):self.step.start()

    def test_return_uses_shared_speed_acceleration_and_calibrated_tcp_limits(self):
        self.step.start();self.step.poll()
        plan=self.commands[0]
        model=self.controller.models['right']
        path=TimedJointPath(plan['joint_waypoints_rad'],model.reach+np.linalg.norm(self.tcp[:3,3]),
            tcp_position=lambda q:(model.wrist_transform(q,5.1)@self.tcp)[:3,3],**self.timing)
        np.testing.assert_allclose(plan['time_from_start_s'],path.times)
        self.assertEqual(self.timing['joint_speed_deg_s'],[15.]*3+[50.]*3)
        self.assertEqual(self.timing['joint_acceleration_deg_s2'],[28.]*6)
        self.assertEqual((self.timing['tcp_speed_mm_s'],self.timing['tcp_acceleration_mm_s2']),(60.,120.))
        times=np.linspace(0,path.times[-1],2001)
        xyz=[]
        for t in times:
            q,v,a=path.sample(t)
            self.assertTrue(np.all(np.abs(np.degrees(v))<=np.array(self.timing['joint_speed_deg_s'])+1e-8))
            self.assertTrue(np.all(np.abs(np.degrees(a))<=np.array(self.timing['joint_acceleration_deg_s2'])+1e-8))
            xyz.append((model.wrist_transform(q,5.1)@self.tcp)[:3,3]*1000)
        dt=times[1]-times[0]
        self.assertLessEqual(np.linalg.norm(np.diff(xyz,axis=0)/dt,axis=1).max(),60.+1e-3)
        self.assertLessEqual(np.linalg.norm(np.diff(xyz,n=2,axis=0)/dt**2,axis=1).max(),120.+1e-3)

    def test_return_plan_passes_existing_ros_transport_without_hardware(self):
        from web.tests.test_trajectory_transport import TrajectoryTransportTests
        transport=TrajectoryTransportTests();transport.setUp()
        def start(side,plan,token,**kwargs):
            handle,confirm=transport.adapter.trajectory_start(side,plan,transport.state)
            confirm()
            return handle
        self.controller.start_trajectory=start
        self.step.start()
        self.assertEqual(self.step.poll()['status'],'returning')
        self.assertEqual(len(transport.sent),1)
        self.assertEqual(transport.sent[0]['kind'],'start')
        self.assertEqual(transport.sent[0]['version'],1)
        self.assertEqual(transport.adapter.pending_arm_commands,{})

    def test_return_waits_for_reference_before_starting_existing_arrival_monitor(self):
        self.trajectory_status='running';self.step.start();self.step.poll()
        self.now+=self.arm_policy['timeout_s']+1
        self.assertEqual(self.step.poll()['status'],'returning')
        self.assertIsNone(self.step.return_monitor)
        self.trajectory_status='reference_complete'
        self.assertEqual(self.finish_return()['status'],'carry_ready')

    def test_return_already_at_transport_pose_does_not_send_duplicate_trajectory(self):
        self.joints=np.radians([0.,0.,90.,-70.,0.,-90.])
        self.step.start();self.step.poll()
        self.assertEqual(self.commands,[])
        for _ in range(5):
            self.now+=.1;receipt=self.step.poll()
        self.assertEqual(receipt['status'],'carry_ready')

    def test_trajectory_failure_stops_without_releasing_gripper(self):
        self.step.start()
        self.controller.poll_trajectory=Mock(side_effect=ValueError('Driver stopped'))
        self.assertTrue(self.step.poll()['stop_required'])
        self.assertEqual([c['type'] for c in self.commands],['arm.trajectory','arm.stop'])

    def test_cancel_before_chassis_completion_never_opens_gripper(self):
        self.retreat.poll.return_value={'status':'backing_chassis'}
        self.extract_step.start()
        self.assertEqual(self.extract_step.poll()['status'],'backing_chassis')
        self.assertTrue(self.extract_step.cancel()['stop_required'])
        self.assertEqual([c['type'] for c in self.commands],['arm.stop'])

    def test_feedback_loss_stops_without_claiming_extraction(self):
        self.extract_step.start()
        self.controller.snapshot=lambda:dict(arms={'right':{'joints':[0.]*6,'fresh':False}},body={'height':5.1,'fresh':True})
        result=self.extract_step.poll()
        self.assertTrue(result['stop_required'])
        self.assertNotEqual(result['status'],'withdrawn')

    def test_return_waits_for_all_six_joint_errors_and_existing_settling(self):
        self.step.start()
        self.assertEqual(self.step.poll()['status'],'returning')
        self.joints=np.radians([0.,0.,0.,0.,0.,-87.9])
        self.now+=.1
        self.assertEqual(self.step.poll()['status'],'returning')
        self.joints=np.radians([0.,0.,0.,0.,0.,-89.])
        self.now+=.1
        self.assertEqual(self.step.poll()['status'],'returning')
        self.assertEqual(self.finish_return()['status'],'carry_ready')

    def test_cancel_during_return_stops_without_opening_or_lift_commands(self):
        self.step.start();self.step.poll()
        receipt=self.step.cancel()
        self.assertTrue(receipt['stop_required'])
        self.assertEqual(self.step.poll()['status'],'cancelled')
        self.assertEqual([c['type'] for c in self.commands],['arm.trajectory','arm.stop'])

    def test_rejected_return_never_reports_carry_ready(self):
        self.controller.start_trajectory=Mock(side_effect=ValueError('Driver rejected return'))
        receipt=self.step.start()
        self.assertTrue(receipt['stop_required'])
        self.assertNotEqual(receipt['status'],'carry_ready')

    def test_cancel_while_chassis_moves_brakes_and_never_starts_return(self):
        self.retreat.poll.return_value={'status':'backing_chassis'}
        self.extract_step.start()
        self.assertEqual(self.extract_step.poll()['status'],'backing_chassis')
        self.extract_step.cancel();self.retreat.cancel.assert_called_once()
        self.assertEqual([c['type'] for c in self.commands],['arm.stop'])

    def test_chassis_failure_stops_without_returning_arm(self):
        self.extract_step.start()
        self.retreat.poll.side_effect=ValueError('motion_pose_stale_or_invalid')
        receipt=self.extract_step.poll()
        self.assertTrue(receipt['stop_required'])
        self.assertEqual(receipt['failure_detail'],'ValueError: motion_pose_stale_or_invalid')
        self.retreat.cancel.assert_called_once()
        self.controller.log.assert_called_once_with(
            'stow_for_carry failed: ValueError: motion_pose_stale_or_invalid')
        self.assertEqual([c['type'] for c in self.commands],['arm.stop'])

    def test_real_controller_joint_command_keeps_measured_gripper_and_lift(self):
        from web.tests.test_control import Controller,FakeAdapter,configured
        adapter=FakeAdapter();controller=Controller(configured(),adapter,lambda:100.)
        controller.update_arm('right',np.radians([0.,0.,0.,0.,0.,-80.]).tolist(),-.9,[0.]*6)
        controller.update_body(5.1,[0.]*4,0.,0.,0.)
        command=dict(type='arm.joints',arm='right',joints=[0.,0.,0.,0.,0.,-90.])
        self.assertTrue(controller.submit(command,'offline')['ok'])
        self.assertEqual(len(adapter.calls),1)
        kind,args=adapter.calls[0]
        self.assertEqual(kind,'arm');self.assertEqual(args[0],'right')
        np.testing.assert_allclose(args[1],np.radians(command['joints']))
        self.assertEqual(args[2],-.9)
        self.assertEqual(controller.snapshot()['body']['height'],5.1)

    def test_real_tool_backend_runs_once_after_hold_and_confirms_carry_ready(self):
        from agent.battery_backend import BatteryControllerBackend
        from agent.battery_tools import BatteryExecutionTools
        from agent.config import Config
        from agent.storage import RunStore
        with tempfile.TemporaryDirectory() as directory:
            store=RunStore.create(directory)
            backend=BatteryControllerBackend(self.controller,'owned',store,
                policies={'refinement':{'position_tolerance_mm':[18.]*3,'orientation_tolerance_deg':5.},
                          'coarse':{'arm_policy':self.arm_policy}},
                pick_setup={'timing_limits':self.timing,'T_wrist_from_tool':self.tcp.tolist()},preflight=lambda *a:True,cancelled=lambda:False,
                timeout_s={'execute_coarse_alignment':1.,'execute_battery_approach':1.,'execute_pick':1.})
            try:
                store.state['mission'].update(stage='HOLD_CONFIRMED',plan={'plan_id':'held','arm':'right'})
                tools=BatteryExecutionTools(Config(motion_enabled=True),store,backends=backend)
                self.retreat.poll.return_value={'status':'backing_chassis'}
                tools.execute('stow_for_carry',{'plan_id':'held'},'extract')
                for _ in range(100):
                    if self.retreat.start.called:break
                    time.sleep(.01)
                self.retreat.start.assert_called_once()
                self.assertEqual(self.commands,[])
                self.retreat.poll.return_value={'status':'chassis_stopped'}
                extract_request=store.state['run_id']+':extract'
                for _ in range(100):
                    if backend.entries[extract_request].wait(.01):break
                    time.sleep(.01)
                self.assertEqual(self.commands,[])
                result=tools.execute('get_execution_status',{},'extract_poll')
                self.assertTrue(result['ok'],result)
                self.assertEqual(store.state['mission']['stage'],'EXTRACTED')
                self.assertEqual(backend.status('stow_for_carry',extract_request)['data']['extraction_dx_mm'],0.)
                self.assertEqual(backend.status('stow_for_carry',extract_request)['data']['chassis_retreat_mm'],200.)

                tools.execute('return_to_carry',{'plan_id':'held'},'carry')
                carry_request=store.state['run_id']+':carry'
                for _ in range(100):
                    if len(self.commands)==1:break
                    time.sleep(.01)
                self.assertEqual(len(self.commands),1)
                self.assertFalse(backend.entries[carry_request].wait(.03))
                self.joints=np.radians([.5,-.5,90.2,-70.2,.1,-89.])
                for _ in range(5):
                    self.now+=.1;time.sleep(.03)
                self.assertTrue(backend.entries[carry_request].wait(1.))
                result=tools.execute('get_execution_status',{},'carry_poll')
                self.assertTrue(result['ok'],result)
                self.assertEqual(store.state['mission']['stage'],'CARRY_READY')
                tools.execute('return_to_carry',{'plan_id':'held'},'carry')
                self.assertEqual([c['type'] for c in self.commands],['arm.trajectory'])
                self.assertEqual(backend.status('return_to_carry',carry_request)['data']['target_transport_joints_deg'],[0.,0.,90.,-70.,0.,-90.])
            finally:backend.cancel_and_wait(1.);store.close()

    def test_tool_does_not_extract_before_hold_confirmation(self):
        from agent.battery_tools import BatteryExecutionTools
        from agent.config import Config
        from agent.storage import RunStore
        with tempfile.TemporaryDirectory() as directory:
            store=RunStore.create(directory)
            try:
                store.state['mission'].update(stage='PICK_EXECUTED',plan={'plan_id':'held','arm':'right'})
                backend=SimpleNamespace(available=lambda operation:True,invoke=lambda *a:self.fail('premature motion'))
                tools=BatteryExecutionTools(Config(motion_enabled=True),store,backends=backend)
                self.assertFalse(tools.execute('stow_for_carry',{'plan_id':'held'},'early')['ok'])
                store.state['mission']['stage']='HOLD_CONFIRMED'
                self.assertFalse(tools.execute('return_to_carry',{'plan_id':'held'},'early_carry')['ok'])
                self.assertEqual(self.commands,[])
            finally:store.close()
