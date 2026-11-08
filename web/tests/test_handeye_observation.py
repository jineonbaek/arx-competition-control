import json,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
import handeye_observation as h
from handeye_column import ColumnModel

class ObservationTests(unittest.TestCase):
    def test_metric_observation_needs_matching_color_profile(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(h,"PUBLIC",Path(tmp)):
            folder=Path(tmp)/"camera-live";folder.mkdir()
            image=np.full((480,640),255,np.uint8)
            dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
            image[180:300,260:380]=cv2.aruco.drawMarker(dictionary,1,120)
            filename="d405-v-a0-1.jpg";cv2.imwrite(str(folder/filename),image)
            t=time.time()
            meta=dict(state="online",raw_image_file=filename,captured_at=t,capture_session="fixture",depth={"state":"disabled"})
            p=folder/"d405.json";p.write_text(json.dumps(meta))
            arm=dict(timestamp=t,joints=[0.]*6);spec=dict(dictionary="5X5_100",marker=1,mm=15)
            model=ColumnModel()
            with self.assertRaises(ValueError):h.observe(spec,t-.5,arm,model)
            meta["color_intrinsics"]=dict(width=640,height=480,fx=400,fy=400,cx=320,cy=240,model="none",coeffs=[0]*5)
            p.write_text(json.dumps(meta))
            sample,_=h.observe(spec,t-.5,arm,model)
            self.assertLess(sample["rms_px"],2)
            self.assertGreater(sample["T_camera_from_marker"][2][3],0)
            with self.assertRaises(ValueError):h.observe(dict(spec,marker=3),t-.5,arm,model)
