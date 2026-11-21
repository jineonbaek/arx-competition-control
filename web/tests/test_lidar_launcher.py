import importlib.util,sys,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'scripts'))
import lidar_services as lidar
spec=importlib.util.spec_from_file_location('lidar_launcher',ROOT/'scripts/start-console.py');launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)
launcher.network_status=lambda: ''
launcher.lio_status=lambda: ''
launcher.ensure_lio=lambda *a: ''
class LidarLauncherTests(unittest.TestCase):
 def test_healthy_raw_streams_reused_without_any_start(self):
  with patch.object(lidar,'probe',return_value=''),patch.object(lidar,'drivers',return_value=[['livox_ros_driver2_node','single_bottom_driver']]),patch.object(lidar.subprocess,'run') as run:
   self.assertEqual(lidar.ensure_lidars(),'');run.assert_not_called()
 def test_existing_faulty_driver_is_reported_not_restarted(self):
  with patch.object(lidar,'probe',return_value='点云过期'),patch.object(lidar,'drivers',return_value=[['livox_ros_driver2_node','single_bottom_driver']]),patch.object(lidar.subprocess,'run') as run:
   self.assertIn('点云过期',lidar.ensure_lidars());run.assert_not_called()
 def test_duplicate_driver_refused(self):
  with patch.object(lidar,'drivers',return_value=[['driver'],['driver']]),patch.object(lidar.subprocess,'run') as run:
   self.assertTrue(lidar.ensure_lidars());run.assert_not_called()
 def test_sensor_recovery_never_requests_bottom_or_stops_hardware(self):
  with patch.object(launcher,'ensure_lidars',return_value='') as ensure,patch.object(launcher,'component_status',return_value={'雷达':''}),patch.object(launcher,'state',return_value={}),patch.object(launcher,'stop_owned') as stop:
   self.assertEqual(launcher.restart_faults(selected=['lidars']),{'雷达':''});ensure.assert_called_once();stop.assert_not_called()
 def test_cold_start_calls_driver_only_without_stationary_claim(self):
  import tempfile,os
  from types import SimpleNamespace
  with tempfile.TemporaryDirectory() as t,patch.object(lidar,'ROOT',Path(t)),patch.dict(os.environ,{'XDG_RUNTIME_DIR':t}),patch.object(lidar,'drivers',return_value=[]),patch.object(lidar,'probe',return_value=''),patch.object(lidar.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout='',stderr='')) as run:
   self.assertEqual(lidar.ensure_lidars(),'');cmd=run.call_args.args[0];self.assertEqual(cmd[-1],'--driver-only');self.assertNotIn('--stationary-confirmed',cmd)
 def test_lidar_fault_does_not_hide_other_repairs(self):
  with patch.object(launcher,'recovery_plan',return_value=['web','lidars']):
   self.assertEqual(launcher.choose_recovery(None),(['web'],False))
 def test_foreign_driver_never_shows_green_even_if_topics_fresh(self):
  with patch.object(lidar,'drivers',return_value=[['livox_ros_driver2_node','other_driver']]),patch.object(lidar,'probe',return_value=''):
   self.assertTrue(lidar.lidar_status())
