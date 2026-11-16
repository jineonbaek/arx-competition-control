import unittest,time,json
from pathlib import Path
import numpy as np,cv2
from agent import cabinet_preview_geometry as g
ROOT=Path(__file__).resolve().parents[2]
TIP=json.loads((ROOT/'web/public/cad/battery-grip.json').read_text())['points_object_mm']['tip']
def frame():
    h,w=300,500;yy,xx=np.mgrid[:h,:w];xyz=np.stack([(xx-250)*.001,(yy-150)*.001,np.ones((h,w))],-1)
    rgb=np.full((h,w,3),150,np.uint8);cv2.rectangle(rgb,(155,100),(345,199),(255,0,0),10)
    t=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,1.5],[0,0,0,1.]])
    return dict(rgb=rgb,xyz=xyz,base_from_color=t,id='test',camera_id='d435',image_time=time.time(),arm_time=time.time(),body_time=time.time(),calibration_sha256='a'*64)
class CabinetTests(unittest.TestCase):
    def test_finds_full_red_rectangle_not_floor_circle(self):
        self.assertTrue(hasattr(g,'detect'),'rectangular cabinet marker detector required')
        r=g.detect(frame()['rgb']);self.assertEqual(r['status'],'complete');self.assertEqual(len(r['inner_quad_px']),4)
    def test_clipped_marker_is_only_partial(self):
        f=frame();f['rgb']=np.roll(f['rgb'],-200,axis=1)
        self.assertEqual(g.detect(f['rgb'])['status'],'partial')
    def test_filled_red_patch_not_window(self):
        f=frame();cv2.rectangle(f['rgb'],(155,100),(345,199),(255,0,0),-1)
        self.assertNotEqual(g.detect(f['rgb'])['status'],'complete')
    def test_localized_plane_uses_rim_not_hole_background(self):
        f=frame();f['xyz'][110:190,165:335,2]=2.
        r=g.localize(f);self.assertAlmostEqual(r['center_base_m'][0],1.,places=2);self.assertGreater(r['inside_axis_base'][0],.99)
    def test_missing_depth_and_old_frame_cannot_produce_plan(self):
        f=frame();f['xyz'][:]=0
        with self.assertRaises(ValueError):g.localize(f)
        f=frame();f['image_time']-=20
        with self.assertRaises(ValueError):g.localize(f)
    def test_actual_models_select_upper_middle_and_demo(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());op=g.opening(p,'competition');self.assertEqual(op['slot_id'],'upper_2');self.assertEqual(op['size_mm'],[169.6,79.])
        self.assertEqual(g.opening(p,'demo')['slot_id'],'demo')
    def test_horizontal_template_has_exact_constant_z(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());b=json.loads((ROOT/'config/battery-envelope.json').read_text())
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-1,0,0],'left_axis_base':[0,1,0],'up_axis_base':[0,0,1]}
        r=g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=TIP)
        points=np.array(r['tcp_points_base_mm']);self.assertTrue(np.all(points[:,2]==points[0,2]));self.assertGreater(points[-1,0],points[0,0]);self.assertGreater(points[0,1],0)
        self.assertFalse(r['executable']);self.assertLess(r['leading_insertion_depth_mm'],150.1);self.assertGreater(r['battery_protrusion_mm'],50)
    def test_nonhorizontal_plane_does_not_silently_change_z(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());b=json.loads((ROOT/'config/battery-envelope.json').read_text());a=np.radians(25)
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-np.cos(a),0,-np.sin(a)],'left_axis_base':[0,1,0],'up_axis_base':[-np.sin(a),0,np.cos(a)]}
        with self.assertRaises(ValueError):g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=TIP)
    def test_leading_path_units_are_mm(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());b=json.loads((ROOT/'config/battery-envelope.json').read_text())
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-1,0,0],'left_axis_base':[0,1,0],'up_axis_base':[0,0,1]}
        r=g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=TIP)
        self.assertGreater(r['leading_points_base_mm'][0][0],500)
    def test_back_margin_covers_all_battery_corners_on_tilted_front(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());b=json.loads((ROOT/'config/battery-envelope.json').read_text());a=np.radians(.5)
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-np.cos(a),0,-np.sin(a)],'left_axis_base':[0,1,0],'up_axis_base':[-np.sin(a),0,np.cos(a)]}
        r=g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=TIP)
        end=np.array(r['leading_points_base_mm'][-1]);h=np.array(b['cross_section_xy_mm'])-np.mean(np.array(b['bounds_mm']),axis=0)[:2]
        pts=end+np.column_stack((np.zeros(len(h)),h[:,0],-h[:,1]));depth=(pts-np.array(w['center_base_m'])*1000)@(-np.array(w['outward_normal_base']))
        self.assertLessEqual(depth.max(),r['cad_first_obstacle_depth_mm']-r['back_margin_mm']+1e-6)
    def test_full_tool_offset_reconstructs_checked_battery_envelope(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());b=json.loads((ROOT/'config/battery-envelope.json').read_text());tip=json.loads((ROOT/'web/public/cad/battery-grip.json').read_text())['points_object_mm']['tip'];center=np.array(b['bounds_mm']).mean(0)
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-1,0,0],'left_axis_base':[0,1,0],'up_axis_base':[0,0,1]}
        r=g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=tip)
        R=np.array(r['R_base_from_tool']);body_leading=np.array(r['tcp_end_base_mm'])+R[:,0]*(tip[2]-b['bounds_mm'][0][2])-R[:,1]*(tip[0]-center[0])+R[:,2]*(tip[1]-center[1])
        np.testing.assert_allclose(body_leading,r['leading_points_base_mm'][-1],atol=1e-8)
    def test_measured_aperture_smaller_than_battery_is_rejected(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());b=json.loads((ROOT/'config/battery-envelope.json').read_text())
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-1,0,0],'left_axis_base':[0,1,0],'up_axis_base':[0,0,1],'measured_size_mm':[169.2,74.76]}
        with self.assertRaises(ValueError):g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=TIP)

class ShallowGeometryTests(unittest.TestCase):
    def test_shallow_depth_is_bounded_and_fixed_z(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());b=json.loads((ROOT/'config/battery-envelope.json').read_text())
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-1,0,0],'left_axis_base':[0,1,0],'up_axis_base':[0,0,1]}
        r=g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=TIP,max_insertion_depth_mm=10,preclearance_mm=80.)
        self.assertEqual(r['leading_insertion_depth_mm'],10);self.assertEqual(r['z_range_mm'],0)
        self.assertAlmostEqual(r['stroke_mm'],90.)
        for depth in [0,-1,31,float('nan'),True]:
            with self.assertRaises(ValueError):g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=TIP,max_insertion_depth_mm=depth)
    def test_shallow_mode_still_rejects_aperture_collision(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());b=json.loads((ROOT/'config/battery-envelope.json').read_text())
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-1,0,0],'left_axis_base':[0,1,0],'up_axis_base':[0,0,1],'measured_size_mm':[169.6,74.]}
        with self.assertRaises(ValueError):g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=TIP,max_insertion_depth_mm=10)

class ShallowTiltTests(unittest.TestCase):
    def test_ten_mm_bounds_deepest_corner_not_center_travel(self):
        p=json.loads((ROOT/'config/cabinet-profiles.json').read_text());b=json.loads((ROOT/'config/battery-envelope.json').read_text());a=np.radians(10.)
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-np.cos(a),0,-np.sin(a)],'left_axis_base':[0,1,0],'up_axis_base':[-np.sin(a),0,np.cos(a)]}
        r=g.horizontal_plan(w,g.opening(p,'demo'),b,tip_point_object_mm=TIP,max_insertion_depth_mm=10)
        cross=np.array(b['cross_section_xy_mm'])-np.array(b['bounds_mm']).mean(0)[:2]
        R=np.array(r['R_base_from_tool']);offsets=cross[:,0,None]*R[:,1]-cross[:,1,None]*R[:,2]
        end=np.array(r['leading_points_base_mm'][-1]);depth=(end+offsets-np.array(w['center_base_m'])*1000)@(-np.array(w['outward_normal_base']))
        self.assertLessEqual(float(depth.max()),10.+1e-5)
