import json
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from plan_d_vision import PlanDVision, battery_upper_face


class Controller:
    def __init__(self, busy=False): self.busy=busy
    def snapshot(self): return {'motion_reserved':self.busy}


class Capture:
    def __init__(self, cache, fixture, stale=False):
        self.cache=Path(cache); self.fixture=fixture; self.stale=stale
    def action(self, data):
        assert data == {'op':'capture'}
        with np.load(self.fixture,allow_pickle=False) as f:
            stamp=time.time()-60 if self.stale else time.time()
            np.savez_compressed(self.cache/'frame.npz',id='test-frame',image_time=stamp,rgb=f['rgb'],xyz=f['xyz'])
        return {'id':'test-frame'}


class PlanDPageTests(unittest.TestCase):
    fixture=Path(__file__).parent/'fixtures/d435-partial-black-battery-frame.npz'
    def test_confirmed_face_is_published_as_non_executable_preview(self):
        with tempfile.TemporaryDirectory() as d:
            preview=PlanDVision(Capture(d,self.fixture),Controller())
            preview._run()
            result=preview.snapshot()
        self.assertEqual(result['status'],'detected')
        self.assertTrue(result['preview_only'])
        self.assertFalse(result['insertion_axis_available'])
        self.assertTrue(result['image'].startswith('data:image/jpeg;base64,'))
        self.assertAlmostEqual(result['metrics']['depth_m'],.303,delta=.01)
        json.dumps(result,allow_nan=False)

    def test_stale_frame_never_publishes_success_or_image(self):
        with tempfile.TemporaryDirectory() as d:
            preview=PlanDVision(Capture(d,self.fixture,stale=True),Controller())
            preview._run();result=preview.snapshot()
        self.assertEqual(result['status'],'unavailable')
        self.assertIsNone(result['image'])

    def test_active_robot_task_blocks_new_capture(self):
        preview=PlanDVision(None,Controller(busy=True))
        with self.assertRaises(ValueError):preview.refresh()
        self.assertFalse(preview.snapshot()['running'])


if __name__=='__main__':unittest.main()
