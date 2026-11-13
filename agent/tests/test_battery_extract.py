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
        self.controller=SimpleNamespace(
            motion_valid=lambda token:self.owned,
            models={'right':SimpleNamespace(wrist_transform=lambda q,h:self.pose.copy())},
            snapshot=lambda:dict(arms={'right':{'joints':[0.]*6,'fresh':True}},body={'height':5.1,'fresh':True}),
            interrupt_motion=lambda token:setattr(self,'owned',False),
            submit=lambda command,token:self.commands.append(command) or {'ok':True})
        self.step=WristExtractionStep(self.controller,'owned',
            refinement={'position_tolerance_mm':[18.,18.,18.],'orientation_tolerance_deg':5.})

    def test_one_wrist_command_subtracts_200_mm_and_preserves_yz_and_pose(self):
        self.assertEqual(self.commands,[])
        self.assertEqual(self.step.start()['status'],'withdrawing')
        command=self.commands[0]
        self.assertEqual(command['type'],'arm.wrist')
        np.testing.assert_allclose(command['position_mm'],[250.,-150.,700.])
        np.testing.assert_allclose(command['orientation_deg'],[-85.,1.,2.])
        self.pose[:3,3]+=[-.2,.017,-.017]
        self.pose[:3,:3]=self.pose[:3,:3]@Rotation.from_euler('x',4.9,degrees=True).as_matrix()
        self.assertEqual(self.step.poll()['status'],'withdrawn')
        self.assertEqual(len(self.commands),1)
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

    def test_real_tool_backend_runs_once_after_hold_and_confirms_carry_ready(self):
        from agent.battery_backend import BatteryControllerBackend
        from agent.battery_tools import BatteryExecutionTools
        from agent.config import Config
        from agent.storage import RunStore
        with tempfile.TemporaryDirectory() as directory:
            store=RunStore.create(directory)
            backend=BatteryControllerBackend(self.controller,'owned',store,
                policies={'refinement':{'position_tolerance_mm':[18.]*3,'orientation_tolerance_deg':5.}},
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
                self.assertTrue(backend.entries[request].wait(1.))
                result=tools.execute('get_execution_status',{},'poll')
                self.assertTrue(result['ok'],result)
                self.assertEqual(store.state['mission']['stage'],'CARRY_READY')
                tools.execute('stow_for_carry',{'plan_id':'held'},'extract')
                self.assertEqual(len(self.commands),1)
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
