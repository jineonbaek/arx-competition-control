"""Extraction uses the manual wrist interface; all feedback and commands are inert."""
import unittest
import tempfile
import time
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from agent.battery_extract import WristExtractionStep


class ExtractionTests(unittest.TestCase):
    def setUp(self):
        self.pose=np.eye(4);self.pose[:3,3]=[.45,-.15,.70]
        self.pose[:3,:3]=Rotation.from_euler('xyz',[-85,1,2],degrees=True).as_matrix()
        self.commands=[];self.owned=True
        self.now=10.
        self.joints=np.radians([10.,20.,30.,5.,6.,-80.])
        self.arm_policy=dict(tolerance=[.035]*6,max_speed=[.05]*6,
            settle_s=.25,timeout_s=20.,stall_s=2.,max_age_s=.3)
        self.controller=SimpleNamespace(
            clock=lambda:self.now,
            motion_valid=lambda token:self.owned,
            models={'right':SimpleNamespace(wrist_transform=lambda q,h:self.pose.copy())},
            snapshot=lambda:dict(arms={'right':{'joints':self.joints.tolist(),'received':self.now,
                'gripper':-1.,'fresh':True}},body={'height':5.1,'fresh':True}),
            interrupt_motion=lambda token:setattr(self,'owned',False),
            submit=lambda command,token:self.commands.append(command) or {'ok':True})
        self.step=WristExtractionStep(self.controller,'owned',
            refinement={'position_tolerance_mm':[18.,18.,18.],'orientation_tolerance_deg':5.},
            arm_policy=self.arm_policy)

    def finish_return(self):
        self.joints=np.radians([.5,-.5,.2,-.2,.1,-89.])
        for _ in range(4):
            self.now+=.1
            result=self.step.poll()
        return result

    def test_one_wrist_command_subtracts_200_mm_and_preserves_yz_and_pose(self):
        self.assertEqual(self.commands,[])
        self.assertEqual(self.step.start()['status'],'withdrawing')
        command=self.commands[0]
        self.assertEqual(command['type'],'arm.wrist')
        np.testing.assert_allclose(command['position_mm'],[250.,-150.,700.])
        np.testing.assert_allclose(command['orientation_deg'],[-85.,1.,2.])
        self.pose[:3,3]+=[-.2,.017,-.017]
        self.pose[:3,:3]=self.pose[:3,:3]@Rotation.from_euler('x',4.9,degrees=True).as_matrix()
        self.assertEqual(self.step.poll()['status'],'returning')
        self.assertEqual(self.commands[1],dict(type='arm.joints',arm='right',joints=[0.,0.,0.,0.,0.,-90.]))
        self.assertEqual(self.finish_return()['status'],'carry_ready')
        self.assertEqual(self.step.poll()['status'],'carry_ready')
        self.assertEqual([c['type'] for c in self.commands],['arm.wrist','arm.joints'])
        with self.assertRaises(ValueError):self.step.start()

    def test_outside_existing_tolerance_waits_and_cancel_never_opens_gripper(self):
        self.step.start();self.pose[0,3]-=.180
        self.assertEqual(self.step.poll()['status'],'withdrawing')
        self.assertTrue(self.step.cancel()['stop_required'])
        self.assertEqual([c['type'] for c in self.commands],['arm.wrist','arm.stop'])

    def test_feedback_loss_stops_without_claiming_extraction(self):
        self.step.start()
        self.controller.snapshot=lambda:dict(arms={'right':{'joints':[0.]*6,'fresh':False}},body={'height':5.1,'fresh':True})
        result=self.step.poll()
        self.assertTrue(result['stop_required'])
        self.assertNotEqual(result['status'],'withdrawn')

    def test_return_waits_for_all_six_joint_errors_and_existing_settling(self):
        self.step.start();self.pose[0,3]-=.2
        self.assertEqual(self.step.poll()['status'],'returning')
        self.joints=np.radians([0.,0.,0.,0.,0.,-87.9])
        self.now+=.1
        self.assertEqual(self.step.poll()['status'],'returning')
        self.joints=np.radians([0.,0.,0.,0.,0.,-89.])
        self.now+=.1
        self.assertEqual(self.step.poll()['status'],'returning')
        self.assertEqual(self.finish_return()['status'],'carry_ready')

    def test_cancel_during_return_stops_without_opening_or_lift_commands(self):
        self.step.start();self.pose[0,3]-=.2;self.step.poll()
        receipt=self.step.cancel()
        self.assertTrue(receipt['stop_required'])
        self.assertEqual(self.step.poll()['status'],'cancelled')
        self.assertEqual([c['type'] for c in self.commands],['arm.wrist','arm.joints','arm.stop'])

    def test_rejected_return_never_reports_carry_ready(self):
        self.step.start();self.pose[0,3]-=.2
        def submit(command,token):
            self.commands.append(command)
            return {'ok':command['type']!='arm.joints'}
        self.controller.submit=submit
        receipt=self.step.poll()
        self.assertTrue(receipt['stop_required'])
        self.assertNotEqual(receipt['status'],'carry_ready')

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
                pick_setup={},preflight=lambda *a:True,cancelled=lambda:False,
                timeout_s={'execute_coarse_alignment':1.,'execute_battery_approach':1.,'execute_pick':1.})
            try:
                store.state['mission'].update(stage='HOLD_CONFIRMED',plan={'plan_id':'held','arm':'right'})
                tools=BatteryExecutionTools(Config(motion_enabled=True),store,backends=backend)
                tools.execute('stow_for_carry',{'plan_id':'held'},'extract')
                for _ in range(100):
                    if self.commands:break
                    time.sleep(.01)
                self.assertEqual(len(self.commands),1)
                self.pose[:3,3]=np.array(self.commands[0]['position_mm'])/1000
                request=store.state['run_id']+':extract'
                for _ in range(100):
                    if len(self.commands)==2:break
                    time.sleep(.01)
                self.assertEqual(len(self.commands),2)
                self.assertFalse(backend.entries[request].wait(.03))
                self.assertNotEqual(store.state['mission']['stage'],'CARRY_READY')
                self.joints=np.radians([.5,-.5,.2,-.2,.1,-89.])
                for _ in range(5):
                    self.now+=.1;time.sleep(.03)
                self.assertTrue(backend.entries[request].wait(1.))
                result=tools.execute('get_execution_status',{},'poll')
                self.assertTrue(result['ok'],result)
                self.assertEqual(store.state['mission']['stage'],'CARRY_READY')
                tools.execute('stow_for_carry',{'plan_id':'held'},'extract')
                self.assertEqual([c['type'] for c in self.commands],['arm.wrist','arm.joints'])
                self.assertEqual(backend.status('stow_for_carry',request)['data']['target_transport_joints_deg'],[0.,0.,0.,0.,0.,-90.])
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
                self.assertEqual(self.commands,[])
            finally:store.close()
