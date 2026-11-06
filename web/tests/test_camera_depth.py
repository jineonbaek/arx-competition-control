import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from camera_depth import write_depth

class DepthTests(unittest.TestCase):
    def test_lossless_depth_and_metadata_use_the_same_immutable_frame(self):
        raw=np.array([[0,1,1000],[2000,65535,40]],dtype=np.uint16)
        with tempfile.TemporaryDirectory() as tmp:
            meta,files=write_depth(Path(tmp),"d405","abc123-7",raw,.0001,
                {"fx":100.,"fy":101.,"cx":1.,"cy":.5},True,7,1234.)
            restored=cv2.imread(str(Path(tmp)/meta["image_file"]),cv2.IMREAD_UNCHANGED)
            np.testing.assert_array_equal(restored,raw)
            self.assertEqual(restored.dtype,np.uint16)
            self.assertEqual(meta["scale_m_per_unit"],.0001)
            self.assertEqual(meta["width"],3)
            self.assertEqual(meta["height"],2)
            self.assertEqual(meta["frame_number"],7)
            self.assertEqual(meta["sensor_timestamp_ms"],1234.)
            self.assertTrue(meta["aligned_to_color"])
            self.assertAlmostEqual(meta["valid_fraction"],5/6)
            self.assertEqual(len(files),2)
            self.assertTrue(all(p.is_file() for p in files))
            json.dumps(meta,allow_nan=False)
    def test_rejects_invalid_depth_without_writing_a_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            for raw,scale in [(np.zeros((2,2),np.float32),.001),(np.zeros((2,2),np.uint16),0),(np.zeros((2,2),np.uint16),float("nan"))]:
                with self.assertRaises(ValueError):
                    write_depth(Path(tmp),"astra","abc123-1",raw,scale,{},False,1,0.)
            self.assertEqual(list(Path(tmp).iterdir()),[])

if __name__=="__main__":unittest.main()
