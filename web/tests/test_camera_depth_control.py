import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from camera_depth_control import DepthControl,KEYS

class DepthControlTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();root=Path(self.temp.name)
        self.control=DepthControl(root/'requests.json',root)
        self.session=self.control.reset()
        for key in KEYS:self.frame(key)
    def tearDown(self):self.temp.cleanup()
    def frame(self,key,**changes):
        data=dict(state='online',capture_session=self.session,updated_at=time.time(),depth={'requested':False,'state':'disabled'})
        data.update(changes);(self.control.frames/(key+'.json')).write_text(json.dumps(data))
    def test_default_off_and_requests_are_independent(self):
        self.assertFalse(any(self.control.enabled(k) for k in KEYS))
        self.control.request({'camera':'d405','enabled':True})
        self.assertTrue(self.control.enabled('d405'))
        self.assertFalse(self.control.enabled('astra'))
        self.assertEqual(self.control.snapshot()['cameras']['d405']['state'],'starting')
        self.frame('d405',depth={'requested':True,'state':'online'})
        self.assertTrue(self.control.snapshot()['cameras']['d405']['active'])
        self.control.request({'camera':'d405','enabled':False})
        status=self.control.snapshot()['cameras']['d405']
        self.assertTrue(status['active']);self.assertEqual(status['state'],'stopping')
        self.frame('d405')
        self.assertEqual(self.control.snapshot()['cameras']['d405']['state'],'disabled')
    def test_calibration_overrides_manual_until_matching_release(self):
        self.control.manual_request(dict(camera="d405",enabled=True))
        token=self.control.acquire_handeye(False)
        self.assertFalse(self.control.enabled("d405"))
        with self.assertRaises(RuntimeError):
            self.control.manual_request(dict(camera="d405",enabled=True))
        self.control.manual_request(dict(camera="astra",enabled=True))
        self.control.release_handeye("stale")
        self.assertTrue(self.control.snapshot()["cameras"]["d405"]["handeye_priority"])
        self.control.release_handeye(token)
        self.assertTrue(self.control.enabled("d405"))
        self.assertTrue(self.control.enabled("astra"))

    def test_crashed_sample_expires_and_capture_fault_is_respected(self):
        self.control.manual_request(dict(camera="d405",enabled=True))
        self.control.acquire_handeye(False)
        data=self.control._read();data['handeye']['expires']=0;self.control._write(data)
        self.assertTrue(self.control.enabled("d405"))
        token=self.control.acquire_handeye(True)
        self.control.request(dict(camera="d405",enabled=False))
        self.control.release_handeye(token)
        self.assertFalse(self.control.enabled("d405"))

    def test_restart_resets_previous_enabled_request(self):
        self.control.request({'camera':'d405','enabled':True})
        self.control.reset();self.assertFalse(self.control.enabled('d405'))
    def test_invalid_requests_and_offline_enable_are_rejected(self):
        for data in [{'camera':'other','enabled':True},{'camera':'d405','enabled':1},{'camera':'d405','enabled':True,'extra':1},[]]:
            with self.assertRaises(ValueError):self.control.request(data)
        self.frame('d435',state='offline')
        with self.assertRaises(ValueError):self.control.request({'camera':'d435','enabled':True})
        self.control.request({'camera':'d435','enabled':False})
    def test_stale_frames_never_confirm_depth_stopped(self):
        self.frame('d405',updated_at=time.time()-30)
        self.assertEqual(self.control.snapshot()['cameras']['d405']['state'],'unavailable')
        self.assertFalse(self.control.snapshot()['cameras']['d405']['color_online'])

if __name__=='__main__':unittest.main()
