import json
import sys
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from cabinet_capture import CabinetColorCapture


class RejectMotion:
    models={}
    def submit(self,*a,**k):raise AssertionError('no motor access')
    def reserve_motion(self,*a,**k):raise AssertionError('no motion reservation')


class ColorCaptureTests(unittest.TestCase):
    def test_color_capture_builds_immutable_frame_without_depth(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);live=root/'web/public/camera-live';live.mkdir(parents=True)
            name='d435-v-abcd-1.jpg';cv2.imwrite(str(live/name),np.zeros((24,32,3),np.uint8))
            calpath=root/'vision/handeye/results/d435-cad-349622074226.json';calpath.parent.mkdir(parents=True)
            calpath.write_text(json.dumps({'serial':'s','status':'nominal CAD preview; physical accuracy not measured','T_base_from_color_optical':np.eye(4).tolist()}))
            cap=CabinetColorCapture(RejectMotion(),camera='d435',root=root)
            k={'serial':'s','width':32,'height':24,'fx':50.,'fy':50.,'cx':16.,'cy':12.,'coeffs':[0.]*5,'model':'distortion.none'}
            meta={'state':'online','captured_at':100.,'raw_image_file':name,'color_intrinsics':k}
            state={'command_epoch':7,'motion_reserved':False,'chassis_active':False,'arms':{'right':{'fresh':True,'joints':[0.]*6,'gripper':-1.,'timestamp':100.}},'body':{'fresh':True,'height':2.,'timestamp':100.,'head_pitch':0.}}
            f=cap.build_frame(meta,state,now=100.)
            self.assertNotIn('xyz',f);self.assertEqual(f['rgb'].shape,(24,32,3));self.assertEqual(f['command_epoch'],7)
            self.assertEqual(f['camera_serial'],'s');self.assertIn('physical accuracy not measured',f['calibration_status'])
            self.assertEqual(f['right_gripper'],-1.)
            meta['raw_image_file']='../outside.jpg'
            with self.assertRaises(ValueError):cap.build_frame(meta,state,now=100.)

    def test_rejects_stale_or_unpaired_feedback_before_publishing_frame(self):
        cap=CabinetColorCapture(RejectMotion(),camera='d435')
        meta={'captured_at':100.,'state':'online'}
        for state in [{'chassis_active':True},{'motion_reserved':True},{'arms':{'right':{'fresh':False}},'body':{}}]:
            with self.assertRaises(ValueError):cap.build_frame(meta,state,now=100.)
