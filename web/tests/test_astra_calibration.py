import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from astra_calibration import CalibrationCapture

class CalibrationCaptureTests(unittest.TestCase):
    def test_unpatched_sdk_is_rejected_before_ir_initialization(self):
        import ctypes
        from types import SimpleNamespace
        from astra_calibration import capture_pair
        sdk=Mock();fake=SimpleNamespace(openni2=sdk,_openni2=SimpleNamespace(OniDeviceInfo=ctypes.c_int))
        with patch.dict(sys.modules,{'openni':fake}),self.assertRaisesRegex(RuntimeError,'Unsafe OpenNI ABI'):
            capture_pair(Mock(),Path('/not-written'))
        sdk.initialize.assert_not_called()

    def test_only_new_requests_for_this_session_can_open_infrared(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture=CalibrationCapture(Path(tmp),'session')
            with patch('astra_calibration.capture_pair') as acquire:
                self.assertFalse(capture.poll(Mock(),False))
                for request in [{'token':'../escape','session':'session'},{'token':'a'*32,'session':'old'}]:
                    capture.request.write_text(json.dumps(request))
                    self.assertFalse(capture.poll(Mock(),False))
                acquire.assert_not_called()

    def test_depth_busy_never_opens_another_astra_stream_or_retries_request(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture=CalibrationCapture(Path(tmp),'session')
            capture.request.write_text(json.dumps({'token':'a'*32,'session':'session'}))
            with patch('astra_calibration.capture_pair') as acquire:
                self.assertTrue(capture.poll(Mock(),True))
                self.assertFalse(capture.poll(Mock(),False))
                acquire.assert_not_called()
            self.assertEqual(json.loads(capture.result.read_text())['state'],'error')

    def test_capture_publication_is_atomic_and_failure_does_not_stop_color(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture=CalibrationCapture(Path(tmp),'session')
            for token,error in [('a'*32,None),('b'*32,RuntimeError('IR unavailable'))]:
                capture.request.write_text(json.dumps({'token':token,'session':'session'}))
                with patch('astra_calibration.capture_pair',side_effect=error,return_value={'serial':'synthetic'}) as acquire:
                    self.assertTrue(capture.poll(Mock(),False))
                    acquire.assert_called_once()
                result=json.loads(capture.result.read_text())
                self.assertEqual(result['token'],token)
                self.assertEqual(result['state'],'error' if error else 'captured')

if __name__=='__main__':unittest.main()
