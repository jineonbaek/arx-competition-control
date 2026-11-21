import hashlib,json,time,tempfile,unittest
from pathlib import Path
from unittest.mock import patch,Mock
import cv2,numpy as np
import grasp_preview as gp
from d405_left_edges import measure_left_edges,CANDIDATE

ROOT=Path(__file__).resolve().parents[2]

def scene(left=258):
    rgb=np.full((480,640,3),170,np.uint8)
    cv2.line(rgb,(left-8,45),(440,45),(230,15,15),8)
    cv2.line(rgb,(left-8,45),(left-8,116),(230,15,15),8)
    cv2.rectangle(rgb,(209,180),(486,260),(20,20,20),-1)
    cv2.fillConvexPoly(rgb,np.array([[277,80],[403,80],[486,180],[209,180]]),(48,48,48))
    return rgb

def frame(rgb):
    return dict(id='rgb',capture_mode='color_only',image_time=10.,arm_time=10.,body_time=10.,camera_id='d405',
        calibration_sha256=hashlib.sha256(CANDIDATE.read_bytes()).hexdigest(),rgb=rgb,base_from_color=np.eye(4))

class RGBAlignmentTests(unittest.TestCase):
    def test_real_glare_frame_uses_inner_paint_boundary_not_highlight_or_background(self):
        rgb=cv2.imread(str(Path(__file__).parent/'fixtures/d405-rgb-occluded-depth.jpg'))[:,:,::-1]
        f=frame(rgb);m=measure_left_edges(f,np.eye(4),ROOT)
        # Manually inspected pixels: nose left corner near (284,85), local
        # inner red lip near (282,85). The background is at x=0..264,y=0..52.
        np.testing.assert_allclose(m['comparison_battery_px'],[284,85],atol=4)
        np.testing.assert_allclose(m['comparison_lip_px'],[282,85],atol=4)
        self.assertTrue(m['in_band']);self.assertEqual(m['step_mm'],0.)
        self.assertFalse(m['depth_used'])

    def test_t2_and_independent_y_capture_use_color_even_when_depth_capture_fails(self):
        from fast_visual_level import FastVisualLevel
        from placement_stages import PlacementStages
        color=frame(scene());color.update(joints=np.zeros(6),lift_rad=0.)
        class ColorOnly:
            cache=Path('/unused')
            def live_frame(self,*args,**kwargs):raise AssertionError('depth capture invoked')
            def live_color_frame(self,*args,**kwargs):return color
        capture=ColorOnly()
        level=object.__new__(FastVisualLevel);level.wrist_capture=capture;level.guard=lambda:None
        self.assertIs(level.capture_pair(2.,color_only=True)[1],color)
        stage=object.__new__(PlacementStages);stage.capture=capture;stage.ctrl=Mock()
        stage.ctrl.snapshot.return_value=dict(command_epoch=1,chassis_active=False)
        self.assertIs(stage._paired_frame(color_only=True)[0],color)

    def test_rgb_without_any_depth_returns_signed_full_correction(self):
        # Background dominates the image; only the attached battery's supported
        # trapezoid, and the red rail inner boundary, may provide this estimate.
        result=measure_left_edges(frame(scene(210)),np.eye(4),ROOT)
        self.assertEqual(result['measurement_source'],'rgb_only')
        self.assertFalse(result['depth_used'])
        self.assertGreater(result['step_mm'],15.)
        self.assertAlmostEqual(result['step_mm'],result['raw_error_mm'])
        self.assertFalse(result['in_band'])
        self.assertGreater(np.mean(np.asarray(result['battery_left_px'])[:,0]),200.)
        opposite=measure_left_edges(frame(scene(310)),np.eye(4),ROOT)
        self.assertLess(opposite['step_mm'],-5.)
        self.assertFalse(opposite['in_band'])

    def test_occluded_depth_arrays_never_change_rgb_result(self):
        f=frame(scene());one=measure_left_edges(f,np.eye(4),ROOT)
        f.update(xyz=np.full((480,640,3),np.nan),raw_depth=np.zeros((480,640),np.uint16))
        two=measure_left_edges(f,np.eye(4),ROOT)
        self.assertEqual(one['clearance_mm'],two['clearance_mm'])
        self.assertEqual(one['rim_left_px'],two['rim_left_px'])

    def test_missing_battery_or_red_rail_does_not_invent_a_correction(self):
        for rgb in (np.full((480,640,3),170,np.uint8),np.zeros((480,640,3),np.uint8)):
            with self.assertRaises(ValueError):measure_left_edges(frame(rgb),np.eye(4),ROOT)
        with self.assertRaises(ValueError):
            measure_left_edges(frame(scene()),np.eye(4),ROOT,after_image_time=10.)

    def test_color_capture_never_acquires_depth_and_returns_no_point_cloud(self):
        cal=json.loads(CANDIDATE.read_text());meta=dict(color_intrinsics=dict(serial=cal['camera']['serial']),captured_at=5.)
        jpeg=cv2.imencode('.jpg',np.zeros((8,8,3),np.uint8))[1].tobytes()
        preview=gp.GraspPreview(Mock(),Mock());preview.model=Mock()
        preview.model.wrist_transform.return_value=np.eye(4)
        self.assertTrue(callable(getattr(preview,'live_color_frame',None)),'RGB capture path missing')
        with patch.object(preview,'read_pair',return_value=(meta,jpeg,None,np.zeros(6),5.,0.,5.)):
            result=preview.live_color_frame()
        self.assertEqual(result['capture_mode'],'color_only')
        self.assertNotIn('xyz',result);self.assertNotIn('raw_depth',result)
        preview.depth.acquire_grasp.assert_not_called();preview.depth.release_handeye.assert_not_called()

    def test_color_pair_accepts_online_color_with_unavailable_depth(self):
        preview=gp.GraspPreview(Mock(),Mock())
        preview.ctrl.adapter.body_calibrated.return_value=True
        def snapshot():
            t=time.time();return dict(chassis_active=False,arms=dict(right=dict(fresh=True,joints=[0]*6,timestamp=t)),
                body=dict(fresh=True,height=0.,timestamp=t))
        preview.ctrl.snapshot.side_effect=snapshot
        with tempfile.TemporaryDirectory() as tmp,patch.object(gp,'LIVE',Path(tmp)):
            p=Path(tmp);name='d405-v-abcdef-1.jpg';(p/name).write_bytes(b'color')
            def metadata(*args,**kwargs):
                return json.dumps(dict(state='online',captured_at=time.time(),raw_image_file=name,
                    depth=dict(state='error',message='occluded')))
            with patch.object(Path,'read_text',side_effect=metadata):
                self.assertIn('color_only',__import__('inspect').signature(preview.read_pair).parameters,
                              'color-only pairing missing')
                result=preview.read_pair(timeout_s=1.,color_only=True)
            self.assertEqual(result[1],b'color');self.assertIsNone(result[2])
        preview.depth.enabled.assert_not_called()
