import unittest
from pathlib import Path
import cv2,numpy as np
from agent.cabinet_preview_geometry import detect

class RedEdgesTests(unittest.TestCase):
    def test_perspective_sides_need_not_be_parallel_in_the_image(self):
        with np.load(Path(__file__).parent/'fixtures/cabinet-left-bay-occluded.npz') as f:rgb=f['rgb']
        result=detect(rgb)
        self.assertEqual(result['status'],'complete',result)
        self.assertEqual(result['observed_edges'],4)
    def test_current_rim_with_battery_covering_corner_rejects_rounded_edge_outliers(self):
        with np.load(Path(__file__).parent/'fixtures/cabinet-large-corner.npz') as f:rgb=f['rgb']
        result=detect(rgb)
        self.assertEqual(result['status'],'complete',result)
        self.assertEqual(result['observed_edges'],4)
        self.assertTrue(all(e['support_pixels']>=12 for e in result['edge_evidence']))
    def test_large_corner_occlusion_uses_four_distributed_inner_edges(self):
        rgb=np.full((300,640,3),220,np.uint8)
        outer=np.array([[160,80],[460,90],[430,240],[140,230]],np.int32)
        inner=np.array([[175,95],[440,104],[416,225],[157,216]],np.int32)
        cv2.fillPoly(rgb,[outer],(230,0,0));cv2.fillPoly(rgb,[inner],(220,220,220))
        rgb[170:260,300:480]=0
        result=detect(rgb)
        self.assertEqual(result['status'],'complete',result)
        self.assertEqual(result['observed_edges'],4)
        self.assertLess(np.max(np.linalg.norm(np.array(result['inner_quad_px'])-inner,axis=1)),2.)
    def test_real_battery_occlusion_fits_visible_red_lines_without_closed_ring(self):
        with np.load(Path(__file__).parent/'fixtures/cabinet-partial-rim.npz') as f:rgb=f['occluded']
        result=detect(rgb)
        self.assertEqual(result['status'],'complete',result)
        expected=np.array([[386.13,39.95],[517.93,28.02],[503.22,83.31],[380.90,96.19]])
        self.assertLess(np.max(np.linalg.norm(np.array(result['inner_quad_px'])-expected,axis=1)),2.)
        self.assertEqual(result['observed_edges'],4)

    def test_entire_missing_edge_does_not_get_invented(self):
        rgb=np.full((240,320,3),220,np.uint8)
        cv2.rectangle(rgb,(70,60),(250,180),(230,0,0),9)
        rgb[165:]=220
        self.assertNotEqual(detect(rgb)['status'],'complete')

    def test_two_partial_frames_are_ambiguous(self):
        rgb=np.full((300,640,3),220,np.uint8)
        for x in [60,360]:
            cv2.rectangle(rgb,(x,60),(x+180,180),(230,0,0),9)
            rgb[170:190,x+60:x+120]=220
        self.assertEqual(detect(rgb)['status'],'ambiguous')
