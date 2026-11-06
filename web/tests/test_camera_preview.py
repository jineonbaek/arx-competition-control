import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))

class PreviewTests(unittest.TestCase):
    def test_atomic_frames_and_offline_state_do_not_reuse_old_image(self):
        self.assertIsNotNone(importlib.util.find_spec('camera_preview'),'Camera preview publisher not implemented')
        from camera_preview import publish, select_device, SPECS
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp)
            publish(d,'d435',b'\xff\xd8frame\xff\xd9',7,100.,'online')
            self.assertEqual((d/'d435.jpg').read_bytes(),b'\xff\xd8frame\xff\xd9')
            meta=json.loads((d/'d435.json').read_text())
            self.assertEqual(meta['sequence'],7)
            self.assertEqual(meta['captured_at'],100.)
            publish(d,'d435',None,7,None,'offline')
            self.assertEqual(json.loads((d/'d435.json').read_text())['state'],'offline')
            self.assertFalse((d/'d435.jpg').exists())
        records=[dict(device='/dev/video40',product=SPECS[0]['product'],index=0,interface='4-2:1.3'),dict(device='/dev/video0',product=SPECS[0]['product'],index=0,interface='4-2:1.0')]
        self.assertEqual(select_device(SPECS[0],records),'/dev/video40')
        self.assertIsNone(select_device(SPECS[0],records+[records[0]]))
        self.assertIsNone(select_device(SPECS[0],[]))

    def test_other_camera_frames_remain_bound_to_their_metadata(self):
        from camera_preview import publish
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp)
            publish(d,'d405',b'first-frame',1,100.,'online')
            first=json.loads((d/'d405.json').read_text())['image_file']
            publish(d,'d405',b'second-frame',2,101.,'online')
            second=json.loads((d/'d405.json').read_text())['image_file']
            self.assertNotEqual(first,second)
            self.assertEqual((d/first).read_bytes(),b'first-frame')
            self.assertEqual((d/second).read_bytes(),b'second-frame')

if __name__=='__main__':unittest.main()
