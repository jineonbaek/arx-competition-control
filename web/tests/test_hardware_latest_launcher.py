import importlib.util,sys,unittest,tempfile
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'scripts'))
spec=importlib.util.spec_from_file_location('hardware_latest_launcher',ROOT/'scripts/start-console.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
m.network_status=lambda: ''
m.lio_status=lambda: ''
m.ensure_lio=lambda *a: ''
m.ensure_lidars=lambda: ''
m.lidar_status=lambda: ''
class HardwareLatestTests(unittest.TestCase):
 def test_entry_uses_repository_launcher_without_cwd_or_legacy_ros(self):
  p=ROOT/'scripts/start-hardware.sh';self.assertTrue(p.exists());s=p.read_text();self.assertIn('start-console.py',s);self.assertNotIn('ros2 launch',s);self.assertIn('--desktop',s)
 def test_running_old_inode_is_reported(self):
  with tempfile.TemporaryDirectory() as t:
   old=Path(t)/'old';new=Path(t)/'new';old.write_text('old');new.write_text('new')
   with patch.object(m,'owned',return_value=[{'pid':123}]),patch.object(m,'driver_executable',return_value=new,create=True),patch.object(m,'running_executable',return_value=old,create=True):
    self.assertIn('旧版',m.driver_version_fault('body'))
 def test_same_executable_has_no_version_fault(self):
  with tempfile.TemporaryDirectory() as t:
   p=Path(t)/'exe';p.write_text('same')
   with patch.object(m,'owned',return_value=[{'pid':123}]),patch.object(m,'driver_executable',return_value=p,create=True),patch.object(m,'running_executable',return_value=p,create=True):
    self.assertEqual(m.driver_version_fault('left'),'')
 def test_missing_target_is_a_visible_fault_not_an_update_request(self):
  with tempfile.TemporaryDirectory() as t:
   with patch.object(m,'owned',return_value=[{'pid':123}]),patch.object(m,'driver_executable',return_value=Path(t)/'missing'):
    self.assertTrue(m.driver_version_fault('body'));self.assertFalse(m.driver_update_needed('body'))
