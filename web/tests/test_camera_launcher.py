import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('console_launcher',ROOT/'scripts/start-console.py')
launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)

class CameraLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.runtime=patch.object(launcher,'RUNTIME',Path(self.temp.name))
        self.runtime.start()
    def tearDown(self):
        self.runtime.stop();self.temp.cleanup()

    def test_camera_only_does_not_enter_robot_launch_and_returns_failure(self):
        self.assertTrue(hasattr(launcher,'ensure_cameras'),'Camera startup is not integrated')
        for fault in ['', 'D405 未连接']:
            with patch.object(launcher,'ensure_cameras',return_value=fault) as ensure,patch.object(launcher,'launch',side_effect=AssertionError('Robot startup must not run')),patch.object(sys,'argv',['start-console.py','--cameras-only']),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(launcher.main(),int(bool(fault)))
                ensure.assert_called_once()

    def test_self_check_rejects_old_files_and_requires_all_three(self):
        self.assertTrue(hasattr(launcher,'camera_fault'),'Camera health check missing')
        with tempfile.TemporaryDirectory() as tmp,patch.object(launcher,'CAMERA_OUTPUT',Path(tmp)):
            self.assertTrue(launcher.camera_fault())
            for key in ['d435','d405','astra']:
                (Path(tmp)/(key+'.json')).write_text(json.dumps({'state':'online','sequence':1,'captured_at':time.time(),'depth':{'state':'online','captured_at':time.time()}}))
            self.assertEqual(launcher.camera_fault(),'')
            (Path(tmp)/'d405.json').write_text(json.dumps({'state':'online','captured_at':time.time()-60}))
            self.assertIn('D405',launcher.camera_fault())

    def test_fresh_color_cannot_hide_missing_failed_or_stale_depth(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(launcher,'CAMERA_OUTPUT',Path(tmp)):
            for depth in [None,{'state':'offline','message':'Access denied'}, {'state':'online','captured_at':time.time()-60}]:
                for key in ['d435','d405','astra']:
                    (Path(tmp)/(key+'.json')).write_text(json.dumps({'state':'online','captured_at':time.time(),'depth':depth}))
                fault=launcher.camera_fault()
                for name in ['D435','D405','Astra Pro']:self.assertIn(name,fault)
                if depth and depth.get('message'):self.assertIn('Access denied',fault)

    def test_existing_capture_is_reused_and_duplicate_process_is_rejected(self):
        self.assertTrue(hasattr(launcher,'ensure_cameras'),'Camera startup missing')
        args=['/usr/bin/python3',str(ROOT/'web/src/camera_preview.py')]
        with patch.object(launcher,'processes',return_value=[args]),patch.object(launcher,'camera_fault',return_value=''),patch.object(launcher,'start') as start:
            self.assertEqual(launcher.ensure_cameras(),'');start.assert_not_called()
        with patch.object(launcher,'processes',return_value=[args,args]),patch.object(launcher,'start') as start:
            self.assertTrue(launcher.ensure_cameras());start.assert_not_called()

    def test_full_launcher_requests_cameras_before_hardware_preflight(self):
        with patch.object(launcher,'ensure_cameras',return_value='') as ensure,patch.object(launcher,'ARM',Path('/missing-camera-test/arm')),patch.object(launcher,'ensure_can',side_effect=AssertionError('CAN must not run in this test')):
            with self.assertRaises(RuntimeError):launcher.launch()
            ensure.assert_called_once()

if __name__=='__main__':unittest.main()
