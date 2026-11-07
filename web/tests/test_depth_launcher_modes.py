import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('depth_modes_launcher',ROOT/'scripts/start-console.py')
launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)

class LauncherDepthModes(unittest.TestCase):
    def test_explicit_depth_off_is_healthy_but_enabled_failure_is_not(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(launcher,'CAMERA_OUTPUT',Path(tmp)):
            for key in ['d435','d405','astra']:
                (Path(tmp)/(key+'.json')).write_text(json.dumps(dict(state='online',captured_at=time.time(),depth=dict(requested=False,state='disabled'))))
            self.assertEqual(launcher.camera_fault(),'')
            (Path(tmp)/'d405.json').write_text(json.dumps(dict(state='online',captured_at=time.time(),depth=dict(requested=True,state='error',message='USB error'))))
            self.assertIn('D405',launcher.camera_fault())
            self.assertIn('USB error',launcher.camera_fault())
    def test_disabled_recognition_is_not_a_recovery_requirement(self):
        self.assertEqual(launcher.whole_battery_fault(),'')

if __name__=='__main__':unittest.main()
