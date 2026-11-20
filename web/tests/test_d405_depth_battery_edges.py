from pathlib import Path
import unittest
import numpy as np
from d405_battery_edges import detect_face,select_depth_consensus,select_depth_preview


class DepthBatteryEdgesTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).parent/'fixtures/d405-battery-foreground.npz'
        with np.load(path,allow_pickle=False) as f:
            self.rgb=f['rgb'];self.xyz=f['xyz'];self.transform=f['base_from_color']

    def detect(self,rgb=None,xyz=None):
        return detect_face(self.rgb if rgb is None else rgb,xyz=self.xyz if xyz is None else xyz,
            base_from_color=self.transform)

    def test_real_dark_frame_fits_body_face_and_excludes_handle(self):
        result=self.detect();self.assertIsNotNone(result)
        expected=np.array([[274,111],[410,111],[491,200],[211,204]],float)
        self.assertLess(np.max(np.linalg.norm(np.array(result['quad_px'])-expected,axis=1)),8.)
        self.assertTrue(result['handle_excluded']);self.assertGreater(result['depth_support_pixels'],500)
        self.assertGreater(result['measured_background_removed_pixels'],5000)
        self.assertEqual(result['invalid_depth_treated_as'],'unknown')
        self.assertFalse(result['metric_calibration'])

    def test_missing_depth_cannot_generate_a_face(self):
        self.assertIsNone(self.detect(xyz=np.zeros_like(self.xyz)))

    def test_handle_texture_does_not_become_a_body_edge(self):
        rgb=self.rgb.copy();rgb[215:470,310:410]=0
        result=self.detect(rgb=rgb);self.assertIsNotNone(result)
        self.assertLess(max(np.array(result['quad_px'])[:,1]),211.)
        np.testing.assert_allclose(result['quad_px'],self.detect()['quad_px'],atol=.1)

    def test_depth_alone_cannot_invent_four_rgb_edges(self):
        self.assertIsNone(self.detect(rgb=np.full_like(self.rgb,80)))

    def test_consensus_selects_a_real_observed_quad_and_rejects_outliers(self):
        r=self.detect();observations=[r,dict(r,quad_px=(np.array(r['quad_px'])+.3).tolist()),
            dict(r,quad_px=(np.array(r['quad_px'])-.4).tolist()),dict(r,quad_px=(np.array(r['quad_px'])+40).tolist()),None]
        index,result=select_depth_consensus(observations)
        self.assertLess(index,3);self.assertEqual(result['consensus_frames'],3)
        self.assertEqual(result['quad_px'],observations[index]['quad_px'])
        self.assertIsNone(select_depth_consensus([r,None,None,None,None]))

    def test_single_valid_frame_is_usable_without_consensus(self):
        r=self.detect();index,value=select_depth_preview([None,r,None])
        self.assertEqual(index,1);self.assertEqual(value['quad_px'],r['quad_px'])
        self.assertFalse(value['consensus_required']);self.assertEqual(value['tracking'],'single_valid_frame')
        self.assertIsNone(select_depth_preview([None,None,None]))


if __name__=='__main__':unittest.main()

class InsertedBatteryEdgesTests(unittest.TestCase):
    def test_inserted_face_is_detected_from_new_depth_with_frozen_mouth_and_occluded_red(self):
        import cv2
        path=Path(__file__).parent/'fixtures/d405-battery-foreground.npz'
        with np.load(path,allow_pickle=False) as f:rgb=f['rgb'].copy();xyz=f['xyz'];T=f['base_from_color'].copy()
        before=detect_face(rgb,xyz=xyz,base_from_color=T);self.assertIsNotNone(before)
        hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60))
        base=xyz@T[:3,:3].T+T[:3,3];valid=(xyz[:,:,2]>.065)&np.isfinite(xyz).all(2)
        mouth=float(np.median(base[red&valid,0]));quad=np.asarray(before['quad_px'])
        yy,xx=np.indices(valid.shape);H=np.linalg.inv(np.asarray(before['H_unit_face_to_image']))
        projected=np.stack((xx,yy,np.ones_like(xx)),axis=-1)@H.T
        uv=projected[:,:,:2]/projected[:,:,2,None]
        band=valid&(uv[:,:,0]>.05)&(uv[:,:,0]<.95)&(uv[:,:,1]>=0)&(uv[:,:,1]<.15)&(base[:,:,0]<mouth-.005)
        leading=float(np.median(base[band,0]))
        shift=mouth+.150-leading;T[0,3]+=shift;rgb[red]=[40,40,40]
        tracking=dict(axis_base=[1.,0.,0.],mouth_base_m=[mouth,0.,0.],expected_front_base_m=[mouth+.150,0.,0.],quad_px=quad.tolist())
        after=detect_face(rgb,xyz=xyz,base_from_color=T,insertion_tracking=tracking)
        self.assertIsNotNone(after,'battery behind the mouth must remain measurable when red rim is occluded')
        np.testing.assert_allclose(after['quad_px'],before['quad_px'],atol=8.)
        self.assertEqual(after['method'],'insertion_depth_supported_four_rgb_lines')

    def test_frozen_world_mouth_reprojects_with_camera_pose_and_measures_inserted_depth(self):
        import cv2,json
        from placement_stage_geometry import battery_front
        with np.load(Path(__file__).parent/'fixtures/d405-battery-foreground.npz',allow_pickle=False) as f:
            rgb=f['rgb'].copy();xyz=f['xyz'];T=f['base_from_color'].copy()
        h,w=rgb.shape[:2];yy,xx=np.indices((h,w));valid=xyz[:,:,2]>.07;k={}
        for dim,pixels,fkey,ckey in [(0,xx,'fx','cx'),(1,yy,'fy','cy')]:
            focal,center=np.linalg.lstsq(np.c_[xyz[:,:,dim][valid]/xyz[:,:,2][valid],np.ones(valid.sum())],pixels[valid],rcond=None)[0]
            k.update({fkey:float(focal),ckey:float(center)})
        k.update(width=w,height=h,model='distortion.none',coeffs=[0.]*5)
        hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=60))
        base=xyz@T[:3,:3].T+T[:3,3];mouth=float(np.median(base[red&valid,0]))
        frame=dict(rgb=rgb,xyz=xyz,base_from_color=T,intrinsics=json.dumps(k))
        cabinet=dict(inside_axis_base=[1.,0.,0.],height_anchor_base_m=[mouth,0.,0.])
        first=battery_front(frame,cabinet)
        T[0,3]+=mouth+.150-first['point_base_m'][0];frame['rgb'][red]=[40,40,40]
        tracking=dict(axis_base=[1.,0.,0.],mouth_base_m=[mouth,0.,0.],expected_front_base_m=[mouth+.15,0.,0.],quad_px=first['quad_px'])
        after=battery_front(frame,cabinet,outside_range_mm=(-210.,300.),insertion_tracking=tracking)
        self.assertGreaterEqual(after['support_pixels'],40)
        self.assertAlmostEqual((after['point_base_m'][0]-mouth)*1000,150.,delta=5.)
