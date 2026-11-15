import json
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import numpy as np
from agent.tests.test_cabinet_rgb_geometry import rgb_frame

ROOT=Path(__file__).resolve().parents[2]


class ReplayTests(unittest.TestCase):
    def test_cli_replays_historical_rgb_frame_without_controller_or_depth(self):
        with tempfile.TemporaryDirectory() as t:
            directory=Path(t);f=rgb_frame();k=json.loads(f['intrinsics']);k['fy']=650*66*169.6/(141*79);f['intrinsics']=json.dumps(k)
            f['calibration_sha256']=hashlib.sha256((ROOT/'vision/handeye/results/d435-cad-349622074226.json').read_bytes()).hexdigest()
            old=time.time()-86400;f.update(image_time=old,arm_time=old,body_time=old)
            np.savez_compressed(directory/'frame.npz',**f)
            p=subprocess.run([sys.executable,str(ROOT/'scripts/replay_cabinet.py'),str(directory/'frame.npz'),
                '--model','demo','--output',str(directory/'result')],capture_output=True,text=True)
            self.assertEqual(p.returncode,0,p.stderr)
            result=json.loads((directory/'result/result.json').read_text())
            self.assertEqual(result['image_time'],old);self.assertTrue(result['replay']['historical'])
            self.assertGreater(result['replay']['actual_frame_age_s'],86000)
            self.assertFalse(result['executable']);self.assertFalse(result['validity']['valid'])
            self.assertFalse(result['readiness']['execution_ready']);self.assertEqual(result['plan']['z_range_mm'],0)
            self.assertGreater((directory/'result/preview.jpg').stat().st_size,1000)
            self.assertEqual(result['environment_evidence']['status'],'unavailable')
