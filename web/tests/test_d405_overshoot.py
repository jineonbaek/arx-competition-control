"""Regress wrong red-component selection and uncalibrated full RGB movement."""
import contextlib,threading,unittest
from pathlib import Path
from unittest.mock import patch
import cv2,numpy as np
import test_d405_horizontal_flow as flow
from d405_rgb_alignment import left_inner_rail


class RailSelectionTests(unittest.TestCase):
    face={'quad_px':[[277.,80.],[403.,80.],[486.,180.],[209.,180.]]}

    def test_longer_unrelated_red_mark_does_not_replace_cabinet_frame(self):
        rgb=np.full((480,640,3),130,np.uint8)
        cv2.line(rgb,(268,49),(438,49),(220,8,8),7)
        cv2.line(rgb,(268,49),(268,110),(220,8,8),7)
        cv2.line(rgb,(150,40),(150,120),(220,8,8),7)
        points=left_inner_rail(rgb,self.face)
        self.assertGreater(float(np.median(points[:,0])),260.)

    def test_dark_red_frame_is_not_lost_to_bright_distractor(self):
        rgb=np.full((480,640,3),20,np.uint8)
        cv2.line(rgb,(268,49),(438,49),(45,2,2),7)
        cv2.line(rgb,(268,49),(268,110),(45,2,2),7)
        cv2.line(rgb,(150,40),(150,120),(220,8,8),7)
        points=left_inner_rail(rgb,self.face)
        self.assertGreater(float(np.median(points[:,0])),260.)

    def test_left_rail_alone_does_not_require_top_or_bottom(self):
        rgb=np.full((480,640,3),30,np.uint8)
        cv2.line(rgb,(268,49),(268,110),(45,2,2),7)
        points=left_inner_rail(rgb,self.face)
        self.assertAlmostEqual(float(np.median(points[:,0])),272.,delta=2.)

    def test_recorded_overlay_uses_actual_red_frame_near_x240(self):
        # This is the archived diagnostic overlay, not a raw camera frame.
        # Supply its independently saved battery quad; test red-edge selection
        # only. Text and drawn yellow/magenta battery edges are not red rails.
        rgb=cv2.imread(str(Path(__file__).parent/'fixtures/d405-before-overshoot-overlay.jpg'))[:,:,::-1]
        face={'quad_px':[[251.7741,93.5615],[419.7350,96.6304],[473.6911,166.8696],[207.9874,162.6236]]}
        points=left_inner_rail(rgb,face)
        self.assertAlmostEqual(float(np.median(points[:,0])),250.,delta=4.)


    def test_latest_aligned_frame_ignores_red_reflections_inside_the_opening(self):
        rgb=cv2.imread(str(Path(__file__).parent/'fixtures/d405-latest-aligned-overlay.jpg'))[:,:,::-1]
        face={'quad_px':[[276.9471,89.6139],[403.0754,87.0083],[486.6350,167.1129],[212.0345,161.2498]]}
        points=left_inner_rail(rgb,face)
        self.assertAlmostEqual(float(np.median(points[:,0])),282.,delta=5.)


    def test_latest_aligned_frame_requests_no_lateral_move_with_pixel_uncertainty(self):
        from test_d405_rgb_alignment import frame,ROOT
        from d405_left_edges import measure_left_edges
        from d405_battery_edges import projective_geometry
        rgb=cv2.imread(str(Path(__file__).parent/'fixtures/d405-latest-aligned-overlay.jpg'))[:,:,::-1]
        quad=np.array([[276.9471,89.6139],[403.0754,87.0083],[486.6350,167.1129],[212.0345,161.2498]])
        # Keep the raw-frame battery quad from the archived receipt: overlay
        # graphics are not input evidence for refitting the battery face.
        face=dict(quad_px=quad.tolist(),**projective_geometry(quad))
        with patch('d405_rgb_alignment.color_battery_face',return_value=face):
            result=measure_left_edges(frame(rgb),np.eye(4),ROOT)
        self.assertLess(abs(result['pixel_gap']),1.)
        self.assertTrue(result['in_band'])
        self.assertEqual(result['step_mm'],0.)


class RGBMovementTests(unittest.TestCase):
    def test_large_rgb_estimate_cannot_command_full_shift_and_small_steps_are_kept(self):
        for error,expected in [(42.347,5.),(-42.347,-5.),(.6,.3)]:
            with self.subTest(error=error):
                level=flow.D405HorizontalFlowTests().make_level()
                level.guard=lambda:None;level.cancel=threading.Event()
                model=level.ctrl.models['right'];origin=model.wrist_transform(level.ctrl.q,level.height)
                targets=[];seen=[]
                def capture(*a,**kw):
                    seen.append(1);stamp=10. if len(seen)==1 else 12.
                    return None,dict(id=str(stamp),image_time=stamp,joints=level.ctrl.q.copy(),lift_rad=level.height)
                def measure(f,**kw):
                    aligned=len(seen)>1
                    return dict(status='visual_guidance_only',measurement_source='rgb_only',axis_base=[0.,1.,0.],
                        raw_error_mm=0. if aligned else error,in_band=aligned,image_time=f['image_time'],frame_id=f['id'])
                def plan(_,m,q,h,tcp,target,*a,**kw):
                    targets.append(target.copy());end=q.copy();end[0]=target[1,3]
                    return dict(joint_waypoints_rad=[q.tolist(),end.tolist()])
                level.capture_pair=capture
                with patch('fast_visual_level.measure_left_edges',side_effect=measure), \
                     patch('fast_visual_level.continuous_correction',side_effect=plan), \
                     patch('fast_visual_level.write_horizontal_overlay'),patch('time.time',return_value=11.):
                    result=level.auto_d405_horizontal_trim(model,np.eye(4),origin)
                self.assertTrue(result['converged']);self.assertEqual(len(targets),1)
                self.assertAlmostEqual((targets[0][1,3]-origin[1,3])*1000.,expected)
                self.assertEqual(len(seen),2)


if __name__=='__main__':unittest.main()
