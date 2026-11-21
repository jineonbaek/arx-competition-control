import importlib.util,sys,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'scripts'))
spec=importlib.util.spec_from_file_location('ros_launcher',ROOT/'scripts/start-console.py');launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)
launcher.network_status=lambda: ''
launcher.lio_status=lambda: ''
launcher.ensure_lio=lambda *a: ''
# Isolate unrelated sensor startup in the existing component fixtures.
launcher.ensure_lidars=lambda: ''
launcher.lidar_status=lambda: ''
class RosLauncherTests(unittest.TestCase):
 def snapshot(self):
  return dict(ros=dict(connected=True,domain_id='191'),chassis_scheduler=dict(backend='ros',ready=False),capabilities={'lift':'','lift.stop':'','chassis':'not ready','chassis.stop':''})
 def test_scheduler_failure_never_requests_body_restart(self):
  with patch.object(launcher,'check_fault',side_effect=lambda s,n:'not ready' if n=='chassis' else ''),patch.object(launcher,'camera_fault',return_value=''),patch.object(launcher,'scheduler_needs_start',return_value=True,create=True):
   self.assertEqual(launcher.recovery_plan(self.snapshot(),{'can1':'','can3':'','can5':''}),['scheduler'])
 def test_scheduler_only_recovery_does_not_require_bottom(self):
  with patch.object(launcher,'ensure_scheduler',return_value='',create=True) as ensure,patch.object(launcher,'component_status',return_value={'底盘':''}),patch.object(launcher,'state',return_value=self.snapshot()),patch.object(launcher,'stop_owned') as stop,patch.object(launcher,'can_health') as can:
   self.assertEqual(launcher.restart_faults(selected=['scheduler']),{'底盘':''});ensure.assert_called_once();stop.assert_not_called();can.assert_not_called()
 def test_start_scheduler_before_camera_or_hardware_checks(self):
  events=[]
  with patch.object(launcher,'request_gamepad_start',return_value=''),patch.object(launcher,'ensure_scheduler',side_effect=lambda:events.append('scheduler'),create=True),patch.object(launcher,'ensure_cameras',side_effect=lambda:(_ for _ in ()).throw(RuntimeError('camera stop'))):
   with self.assertRaises(RuntimeError):launcher.launch()
  self.assertEqual(events,['scheduler'])
 def test_service_start_is_idempotent_and_never_calls_hardware(self):
  calls=[]
  def run(args,**kw):calls.append(args);return SimpleNamespace(returncode=0,stdout='',stderr='')
  with patch.object(launcher.subprocess,'run',side_effect=run),patch.object(launcher,'start') as start,patch.object(launcher,'stop_owned') as stop:
   launcher.ensure_scheduler();launcher.ensure_scheduler();start.assert_not_called();stop.assert_not_called()
  self.assertTrue(all(c[:2]==['systemctl','--user'] for c in calls));self.assertFalse(any('restart' in c for c in calls))
 def test_false_ready_overrides_empty_capability(self):
  s=self.snapshot();s['ros']['feedback_sources']={'body':dict(status='fresh',publishers=1,other_publishers=0)};s['capabilities']['chassis']=''
  self.assertIn('ROS',launcher.check_fault(s,'chassis'))
 def test_cold_scheduler_failure_keeps_camera_and_web_path_available(self):
  with patch.object(launcher,'request_gamepad_start',return_value=''),patch.object(launcher,'ensure_scheduler',side_effect=RuntimeError('scheduler failed')),patch.object(launcher,'ensure_cameras',side_effect=RuntimeError('camera reached')),patch.object(launcher,'SCHEDULER_START_WARNING','',create=True):
   with self.assertRaisesRegex(RuntimeError,'camera reached'):launcher.launch()
 def test_cold_missing_scheduler_is_repaired_before_web(self):
  with patch.object(launcher,'scheduler_needs_start',return_value=True),patch.object(launcher,'camera_fault',return_value=''):
   self.assertEqual(launcher.recovery_plan(None),['scheduler','web'])
 def test_transient_binding_wait_has_no_hardware_side_effect(self):
  old=self.snapshot();new=self.snapshot();new['chassis_scheduler']['ready']=True
  with patch.object(launcher,'check_fault',return_value=''),patch.object(launcher,'scheduler_needs_start',return_value=False),patch.object(launcher,'wait_for',return_value=new) as wait,patch.object(launcher,'stop_owned') as stop:
   self.assertIs(launcher.wait_scheduler_binding(old),new);wait.assert_called_once();stop.assert_not_called()
 def test_hardware_fault_does_not_wait_for_scheduler_binding(self):
  with patch.object(launcher,'check_fault',return_value='hardware absent'),patch.object(launcher,'wait_for') as wait:
   launcher.wait_scheduler_binding(self.snapshot());wait.assert_not_called()
