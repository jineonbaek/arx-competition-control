import json,sys,unittest,time
from pathlib import Path
import numpy as np
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from d435_cad import CAD,locate_tags,board_center

class D435PreviewTests(unittest.TestCase):
    def test_known_tag_in_color_optical_frame(self):
        cad=json.loads(CAD.read_text())
        image=np.full((480,640,3),255,np.uint8)
        tag=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),12,101)
        image[190:291,270:371]=tag[:,:,None]
        stamp=time.time()
        k=dict(serial=cad['serial'],width=640,height=480,fx=600,fy=600,cx=320,cy=240,coeffs=[0]*5)
        ids,rows,board=locate_tags(image,k,cad,dict(fresh=False),stamp)
        self.assertIsNone(board)
        self.assertEqual(ids,[12])
        expected=np.array(cad['T_base_from_color_optical'])@np.array([0,0,.33,1])
        np.testing.assert_allclose(rows[0]['base_mm'],expected[:3]*1000,atol=.6)
        self.assertIsNone(rows[0]['body4_mm'])

    def test_partial_grid_center(self):
        ids = [0, 5, 14, 21, 30, 35]
        pixels = {i: [10 + 32*(i%6), 270 - 31*(i//6)] for i in ids}
        tags = [dict(id=i, base_mm=[800, (i%6)*70., (i//6)*70.]) for i in ids]
        self.assertEqual(board_center(tags, pixels), dict(base_mm=[800.,175.,175.], pairs=3))
        self.assertIsNone(board_center(tags, {0: pixels[0], 35: pixels[35]}))
        pixels[21] = [20., 20.]
        self.assertIsNone(board_center(tags, pixels))

    def test_touching_corners_on_aprilgrid(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        gray = np.full((480, 640), 245, np.uint8)
        size, gap = 35, 10
        pitch = size + gap
        for row in range(7):
            for col in range(7):
                y, x = 20 + row*pitch, 100 + col*pitch
                gray[y:y+gap, x:x+gap] = 0
                if row < 6 and col < 6:
                    gray[y+gap:y+pitch, x+gap:x+pitch] = cv2.aruco.drawMarker(
                        dictionary, 30 - 6*row + col, size)
        rotation = cv2.getRotationMatrix2D((320, 240), 2, 1)
        gray = cv2.warpAffine(gray, rotation, (640, 480), borderValue=255)
        native_count = len(cv2.aruco.detectMarkers(gray, dictionary)[0])
        cad = json.loads(CAD.read_text())
        k = dict(serial=cad['serial'], width=640, height=480, fx=600, fy=600,
                 cx=320, cy=240, coeffs=[0]*5)
        _, tags, board = locate_tags(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), k, cad, {}, 0)
        self.assertGreater(len(tags), native_count)
        self.assertIsNotNone(board)
