import json,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
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
            near_q=np.radians([0,0,0,0,0,7.3])
            paired,_=h.observe(spec,t-.5,arm,model,[(t-.14,np.zeros(6)),(t-.008,near_q),(t+.08,np.zeros(6))])
            self.assertEqual(paired["joints_rad"],near_q.tolist())
            self.assertEqual(paired["arm_time"],t-.008)
            self.assertLess(sample["rms_px"],2)
            self.assertGreater(sample["T_camera_from_marker"][2][3],0)
            with self.assertRaises(ValueError):h.observe(dict(spec,marker=3),t-.5,arm,model)

    def test_board_pose_keeps_marker26_frame_when_reference_is_hidden(self):
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        image=np.full((480,640),255,np.uint8)
        k=dict(height=480,width=640,fx=600,fy=600,cx=320,cy=240,model="none",coeffs=[0]*5)
        spec=h.board_spec(dict(dictionary="5X5_100",marker=26,mm=11.25,layout="CC200-15-11.25"))
        for marker in range(54):
            row,col=divmod(marker,6)
            x=160+32*(2*col+(row+1)%2);y=90+32*row
            image[y:y+24,x:x+24]=cv2.aruco.drawMarker(dictionary,marker,24)
        single,_,_=h.estimate_target(image,k,dict(spec,layout=None))
        x=160+32*5;y=90+32*4
        image[y:y+24,x:x+24]=255
        image[90:114,192:216]=cv2.aruco.drawMarker(dictionary,53,24)
        board,rms,ids=h.estimate_target(image,k,spec)
        self.assertNotIn(26,ids);self.assertNotIn(53,ids)
        self.assertGreater(len(ids),40);self.assertLess(rms,1)
        np.testing.assert_allclose(single[:3,3],board[:3,3],atol=.013)
        np.testing.assert_allclose(board[:3,:3],np.diag([1,-1,-1]),atol=.002)
        image[:90+4*32]=255;image[90+4*32+24:]=255
        with self.assertRaises(ValueError):h.estimate_target(image,k,spec)
