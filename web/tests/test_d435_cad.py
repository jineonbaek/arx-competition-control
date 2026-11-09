import json,sys,unittest,time
from pathlib import Path
import numpy as np
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from d435_cad import CAD,locate_tags

class D435PreviewTests(unittest.TestCase):
    def test_known_tag_in_color_optical_frame(self):
        cad=json.loads(CAD.read_text())
        image=np.full((480,640,3),255,np.uint8)
        tag=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),12,101)
        image[190:291,270:371]=tag[:,:,None]
        stamp=time.time()
        k=dict(serial=cad['serial'],width=640,height=480,fx=600,fy=600,cx=320,cy=240,coeffs=[0]*5)
        ids,rows=locate_tags(image,k,cad,dict(fresh=False),stamp)
        self.assertEqual(ids,[12])
        expected=np.array(cad['T_base_from_color_optical'])@np.array([0,0,.33,1])
        np.testing.assert_allclose(rows[0]['base_mm'],expected[:3]*1000,atol=.6)
        self.assertIsNone(rows[0]['body4_mm'])
