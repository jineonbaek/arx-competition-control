import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
import navigation_services as nav
class LioTests(unittest.TestCase):
 def test_healthy_existing_lio_is_not_reinitialized(self):
  with patch.object(nav,'lio_processes',return_value=[123]),patch.object(nav,'lio_status',return_value=''),patch.object(nav.subprocess,'run') as run:
   self.assertEqual(nav.ensure_lio(False),'');run.assert_not_called()
 def test_cold_lio_requires_explicit_stationary_confirmation(self):
  with patch.object(nav,'lio_processes',return_value=[]),patch.object(nav.subprocess,'run') as run:
   self.assertIn('静止',nav.ensure_lio(False));run.assert_not_called()
 def test_faulty_existing_lio_is_not_stopped(self):
  with patch.object(nav,'lio_processes',return_value=[123]),patch.object(nav,'lio_status',return_value='odometry stale'),patch.object(nav.subprocess,'run') as run:
   self.assertIn('保留',nav.ensure_lio(True));run.assert_not_called()
 def test_network_wrong_route_is_a_fault(self):
  from types import SimpleNamespace
  replies=[SimpleNamespace(returncode=0,stdout='[{"flags":["UP"],"addr_info":[{"local":"192.168.1.50"}]}]'),SimpleNamespace(returncode=0,stdout='[{"dev":"wlan0"}]')]
  with patch.object(nav.subprocess,'run',side_effect=replies),patch.object(nav,'enabled_ips',return_value=['192.168.1.3']):
   self.assertIn('路由',nav.check_network())
 def test_arm_only_reservation_does_not_imply_base_motion(self):
  import json
  from unittest.mock import MagicMock
  s=dict(body={'fresh':True},chassis_scheduler={'ready':True},nav_sdk_ready=True,chassis_active=False,motion_reserved=True,competition_reserved=False)
  with patch.object(nav.urllib.request,'urlopen',return_value=MagicMock()),patch.object(nav.json,'load',return_value=s):self.assertTrue(nav.robot_idle())
 def test_stale_scheduler_never_counts_as_idle(self):
  from unittest.mock import MagicMock
  s=dict(body={'fresh':True},chassis_scheduler={'ready':False},nav_sdk_ready=False,chassis_active=False,competition_reserved=False)
  with patch.object(nav.urllib.request,'urlopen',return_value=MagicMock()),patch.object(nav.json,'load',return_value=s):self.assertFalse(nav.robot_idle())
 def test_duplicate_raw_driver_blocks_cold_init(self):
  import lidar_services,tempfile,os
  with tempfile.TemporaryDirectory() as t,patch.dict(os.environ,{'XDG_RUNTIME_DIR':t}),patch.object(nav,'lio_processes',return_value=[]),patch.object(nav,'robot_idle',return_value=True),patch.object(nav,'network_status',return_value=''),patch.object(lidar_services,'lidar_status',return_value='重复驱动'),patch.object(nav.subprocess,'run') as run:
   self.assertIn('重复',nav.ensure_lio(True));run.assert_not_called()
 def test_pure_odometry_launch_has_no_grid_or_map_converter(self):
  p=Path(__file__).resolve().parents[2]/'navigation/single_bottom/odometry.launch.py';s=p.read_text()
  self.assertNotIn('pointmap_grid.py',s);self.assertNotIn("['body','map']",s);self.assertIn('map_en=False',s)
