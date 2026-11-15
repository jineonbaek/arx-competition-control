from contextlib import nullcontext
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'web/src'))
from cabinet_verification import verify_placement
from agent.storage import RunStore


class CabinetVerificationTests(unittest.TestCase):
    def fixture(self,*,moving=False,wrong=False):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        store=RunStore.create(Path(tmp.name));self.addCleanup(store.close)
        texture=np.random.default_rng(12).integers(0,255,(200,200,3),dtype=np.uint8)
        calls=[]
        def capture(**kwargs):
            n=len(calls);calls.append(n)
            rgb=np.roll(texture,n*3 if moving else 0,axis=1)
            return dict(rgb=rgb,id=str(n),image_time=10.+n*.5)
        def project(frame,points):return [[float(p[1])+100,float(p[2])+100] for p in points]
        preview=SimpleNamespace(captures={'d435':SimpleNamespace(capture=capture)},
            _project=project,battery={'bounds_mm':[[0.,0.,0.],[80.,75.,195.]]})
        observation=dict(window={'center_base_m':[0.,0.,0.]},
            plan={'R_base_from_tool':[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
                  'leading_points_base_mm':[[0.,0.,0.],[150.,0.,0.]]})
        def recognize(store,ident,output,cancelled):
            mask=np.zeros((200,200),np.uint8)
            if wrong:mask[5:25,5:25]=255
            else:mask[68:132,68:132]=255
            Image.fromarray(mask).save(output/'mask.png')
            return dict(status='perception_accepted')
        return store,preview,observation,recognize,calls

    def run_verify(self,*,moving=False,wrong=False):
        store,preview,observation,recognize,calls=self.fixture(moving=moving,wrong=wrong)
        with patch('cabinet_verification.segment',recognize),patch('cabinet_verification.gpu_slot',nullcontext):
            result=verify_placement(preview,store,observation,object(),lambda:None,lambda dt:None)
        return result,calls

    def test_confirms_actual_recognition_and_new_tracking_frames(self):
        result,calls=self.run_verify()
        self.assertTrue(result['placement_confirmed']);self.assertGreaterEqual(result['stable_duration_s'],3.)
        self.assertGreaterEqual(len(calls),8)

    def test_recognized_battery_elsewhere_is_not_placement_success(self):
        with self.assertRaisesRegex(ValueError,'本次入仓目标'):self.run_verify(wrong=True)

    def test_moving_battery_cannot_pass_stability(self):
        with self.assertRaisesRegex(ValueError,'仍在移动'):self.run_verify(moving=True)


if __name__=='__main__':unittest.main()
