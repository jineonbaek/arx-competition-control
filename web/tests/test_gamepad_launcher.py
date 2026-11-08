import importlib.util
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
import gamepad_services as gamepad

class GamepadLauncherTests(unittest.TestCase):
 def test_start_orders_viewer_before_control_and_never_restarts(self):
  calls=[]
  def run(args,**kwargs):
   calls.append(args);return SimpleNamespace(returncode=0,stdout='',stderr='')
  with patch.object(gamepad.subprocess,'run',side_effect=run):
   self.assertEqual(gamepad.ensure_gamepad_services(),'')
  self.assertEqual(calls,[['systemctl','--user','start','arx-gamepad-viewer.service'],['systemctl','--user','start','arx-gamepad-teleop.service']])
 def test_viewer_failure_does_not_start_control(self):
  with patch.object(gamepad.subprocess,'run',return_value=SimpleNamespace(returncode=1,stdout='',stderr='unit missing')) as run:
   self.assertIn('unit missing',gamepad.ensure_gamepad_services())
   self.assertEqual(run.call_count,1)
 def test_inactive_services_are_visible_without_starting(self):
  with patch.object(gamepad.subprocess,'run',return_value=SimpleNamespace(returncode=3,stdout='inactive',stderr='')) as run:
   result=gamepad.gamepad_status()
   self.assertTrue(all(result.values()))
   self.assertTrue(all('is-active' in call.args[0] for call in run.call_args_list))
 def test_lt_released_is_healthy_when_transport_is_running(self):
  snapshot={'connected':True,'initialized':True,'mode':'XINPUT','error':'','control':{'execute':True,'enabled':False}}
  with patch.object(gamepad,'service_active',return_value=True),patch.object(gamepad,'read_gamepad_state',return_value=snapshot):
   self.assertEqual(gamepad.gamepad_status(),{'手柄可视化':'','手柄底盘控制':''})
 def test_unknown_mode_fault_is_reported(self):
  snapshot={'connected':False,'initialized':False,'mode':None,'error':'未知手柄模式；禁止底盘输出','control':{'execute':False}}
  with patch.object(gamepad,'service_active',return_value=True),patch.object(gamepad,'read_gamepad_state',return_value=snapshot):
   self.assertIn('未知手柄模式',gamepad.gamepad_status()['手柄可视化'])
   self.assertTrue(gamepad.gamepad_status()['手柄底盘控制'])
 def test_missing_device_does_not_request_hardware_restart(self):
  with patch.object(gamepad,'service_active',return_value=True),patch.object(gamepad,'read_gamepad_state',return_value={'connected':False,'error':'手柄已断开'}):
   self.assertFalse(gamepad.gamepad_restart_needed())


spec=importlib.util.spec_from_file_location('launcher_gamepad_integration',ROOT/'scripts/start-console.py')
launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)

class IntegrationTests(unittest.TestCase):
 def test_desktop_launch_starts_handpad_without_restarting_hardware(self):
  with patch.object(launcher,'request_gamepad_start',return_value='') as start,patch.object(launcher,'ensure_cameras',side_effect=RuntimeError('stop before hardware')):
   with self.assertRaises(RuntimeError):launcher.launch()
   start.assert_called_once()
 def test_status_includes_both_services(self):
  faults={'手柄可视化':'viewer failed','手柄底盘控制':'control failed'}
  with patch.object(launcher,'can_health',return_value=({'can1':'','can3':'','can5':''},None)),patch.object(launcher,'check_fault',return_value=''),patch.object(launcher,'camera_fault',return_value=''),patch.object(launcher,'whole_battery_fault',return_value=''),patch.object(launcher,'gamepad_status',return_value=faults):
   result=launcher.component_status({})
   self.assertEqual({key:result[key] for key in faults},faults)
 def test_handpad_recovery_only_starts_user_services(self):
  with patch.object(launcher,'repair_gamepad_services',return_value='') as ensure,patch.object(launcher,'component_status',return_value={'handpad':''}),patch.object(launcher,'state',return_value={}),patch.object(launcher,'stop_owned') as stop,patch.object(launcher,'ensure_can') as can:
   self.assertEqual(launcher.restart_gamepad_services(),{'handpad':''})
   ensure.assert_called_once();stop.assert_not_called();can.assert_not_called()

class RepairRegressionTests(unittest.TestCase):
 def tearDown(self):
  if hasattr(gamepad,'START_ERRORS'):gamepad.START_ERRORS.clear()
 def test_faulty_handpad_does_not_block_hardware_recovery(self):
  with patch.object(launcher,'recovery_plan',return_value=['body']),patch.object(launcher,'gamepad_restart_needed',return_value=True):
   self.assertEqual(launcher.choose_recovery({}),(['body'],False))
 def test_active_but_unreachable_viewer_is_restarted_on_explicit_retry(self):
  with patch.object(gamepad,'service_active',return_value=True),patch.object(gamepad,'read_gamepad_state',side_effect=OSError('connection refused')),patch.object(gamepad.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout='',stderr='')) as run:
   self.assertEqual(gamepad.repair_gamepad_services(),'')
   self.assertEqual(run.call_args_list[0].args[0],['systemctl','--user','restart',gamepad.VIEWER])
 def test_active_hung_transport_is_selected_for_retry(self):
  data={'connected':True,'initialized':True,'mode':'XINPUT','control':{'execute':False}}
  with patch.object(gamepad,'service_active',return_value=True),patch.object(gamepad,'read_gamepad_state',return_value=data):
   self.assertTrue(gamepad.gamepad_restart_needed())
 def test_specific_start_failure_is_shown_in_status(self):
  with patch.object(gamepad.subprocess,'run',return_value=SimpleNamespace(returncode=1,stdout='',stderr='Unit missing at /test')):
   gamepad.ensure_gamepad_services()
  with patch.object(gamepad,'service_active',return_value=False):
   self.assertIn('Unit missing',gamepad.gamepad_status()['手柄可视化'])


class OptionalIsolationTests(unittest.TestCase):
 def test_optional_faults_do_not_fail_global_result(self):
  self.assertFalse(launcher.required_faults({'底盘':'','相机':'','手柄可视化':'not connected','手柄底盘控制':'start failed'}))
  self.assertTrue(launcher.required_faults({'底盘':'CAN offline','手柄可视化':''}))
 def test_unexpected_start_exception_does_not_skip_cameras(self):
  with patch.object(launcher,'request_gamepad_start',side_effect=RuntimeError('systemd failed')),patch.object(launcher,'ensure_cameras',side_effect=RuntimeError('camera phase reached')) as cameras:
   with self.assertRaisesRegex(RuntimeError,'camera phase reached'):launcher.launch()
   cameras.assert_called_once()
 def test_status_exception_does_not_hide_other_components(self):
  with patch.object(launcher,'can_health',return_value=({'can1':'','can3':'','can5':''},None)),patch.object(launcher,'check_fault',return_value=''),patch.object(launcher,'camera_fault',return_value=''),patch.object(launcher,'whole_battery_fault',return_value=''),patch.object(launcher,'gamepad_status',side_effect=RuntimeError('status failed')):
   result=launcher.component_status({})
   self.assertEqual(result['底盘'],'');self.assertEqual(result['相机'],'')
   self.assertIn('status failed',result['手柄可视化'])
 def test_start_is_nonblocking_systemd_request(self):
  with tempfile.TemporaryDirectory() as tmp,patch.object(gamepad.subprocess,'Popen') as popen:
   self.assertEqual(gamepad.request_gamepad_start(Path(tmp)),'')
   self.assertEqual(popen.call_args.args[0],['systemctl','--user','--no-block','start',gamepad.VIEWER,gamepad.CONTROL])


class OptionalResultTests(unittest.TestCase):
 def test_check_only_succeeds_with_handpad_failure_but_rejects_required_fault(self):
  for fault,expected in [('',0),('CAN offline',1)]:
   values={'底盘':fault,'手柄可视化':'未连接','手柄底盘控制':'启动失败'}
   with patch.object(launcher,'component_status',return_value=values),patch.object(launcher,'state',return_value={}),patch.object(sys,'argv',['start-console.py','--check-only']),patch('builtins.print'):
    self.assertEqual(launcher.main(),expected)
 def test_optional_recovery_failure_is_only_an_optional_status(self):
  with patch.object(launcher,'repair_gamepad_services',side_effect=RuntimeError('unit failed')),patch.object(launcher,'state',return_value={}),patch.object(launcher,'component_status',return_value={'底盘':'','相机':'','手柄可视化':'','手柄底盘控制':''}):
   result=launcher.restart_gamepad_services()
   self.assertFalse(launcher.required_faults(result))
   self.assertIn('unit failed',result['手柄可视化'])

if __name__=='__main__':unittest.main()
