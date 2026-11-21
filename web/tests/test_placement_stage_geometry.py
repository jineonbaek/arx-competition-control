import json,time,unittest
import numpy as np
from scipy.spatial.transform import Rotation
from placement_stage_geometry import cabinet_observation,height_reading,z_target

def frame():
    h,w=480,640;yy,xx=np.indices((h,w));z=np.full((h,w),.4)
    xyz=np.stack(((xx-320)/400*z,(yy-240)/400*z,z),axis=2).astype('float32')
    rgb=np.zeros((h,w,3),np.uint8)
    rgb[140:340,270:370]=[230,15,15];rgb[150:330,280:360]=0
    T=np.eye(4);T[:3,:3]=[[0,0,1],[0,-1,0],[1,0,0]];T[:3,3]=[.1,0,.2]
    return dict(rgb=rgb,xyz=xyz,base_from_color=T,id='test',camera_id='d405',
                image_time=time.time(),calibration_sha256='a'*64,joints=np.zeros(6),lift_rad=0.,
                intrinsics=json.dumps(dict(width=w,height=h,fx=400.,fy=400.,cx=320.,cy=240.,model='distortion.none',coeffs=[0.]*5)))

class GeometryTests(unittest.TestCase):
    def test_world_height_cancels_common_base_translation_and_tilt(self):
        f=frame();tool=np.eye(4);tool[:3,3]=[.3,0.,.28]
        obs=cabinet_observation(f);a=height_reading(tool,obs,630.,now=f['image_time'])
        self.assertAlmostEqual(a['tcp_z_mm'],710.,delta=1.)
        Q=Rotation.from_euler('xy',[9.,-12.],degrees=True).as_matrix();shift=np.array([.04,-.02,.075])
        f['base_from_color'][:3,:3]=Q@f['base_from_color'][:3,:3]
        f['base_from_color'][:3,3]=Q@f['base_from_color'][:3,3]+shift
        tool[:3,3]=Q@tool[:3,3]+shift
        b=height_reading(tool,cabinet_observation(f),630.,now=f['image_time'])
        self.assertAlmostEqual(a['tcp_z_mm'],b['tcp_z_mm'],delta=.1)

    def test_missing_opposite_horizontal_edges_does_not_invent_y_center(self):
        f=frame();f['rgb'][330:345,265:375]=0
        obs=cabinet_observation(f)
        self.assertIsNotNone(obs['height_anchor_base_m'])
        self.assertIsNone(obs['lateral_anchor_base_m'])
        self.assertEqual(obs['height_edge_pair'],'left_right')

    def test_no_depth_or_stale_vision_is_not_absolute_height(self):
        f=frame();tool=np.eye(4);obs=cabinet_observation(f)
        with self.assertRaisesRegex(ValueError,'过期'):height_reading(tool,obs,630.,now=f['image_time']+3.)
        f['xyz'][:]=0
        with self.assertRaises(ValueError):cabinet_observation(f)

    def test_z_target_preserves_xy_and_rotation_and_handles_tilt(self):
        T=np.eye(4);T[:3,3]=[.4,-.2,.3];up=np.array([.1,.2,np.sqrt(.95)])
        target=z_target(T,up,10.)
        np.testing.assert_array_equal(target[:2,3],T[:2,3]);np.testing.assert_array_equal(target[:3,:3],T[:3,:3])
        self.assertAlmostEqual(up@(target[:3,3]-T[:3,3])*1000,10.)

    def test_insertion_planner_keeps_height_in_visual_up_basis(self):
        from agent.held_insertion_path import plan_insert
        class Model:
            reach=.7
            def wrist_transform(self,q,h):
                T=np.eye(4);T[:3,3]=q[:3];return T
            def inverse_wrist(self,q,h,xyz,rpy,limits):
                result=np.array(q);result[:3]=np.asarray(xyz)/1000;return result
            def check(self,a,b,h):return {'clear':True,'lower_mm':100.}
        q=np.array([.3,.1,.5,0.,0.,0.]);axis=np.array([np.cos(.04),0.,-np.sin(.04)]);up=np.array([np.sin(.04),0.,np.cos(.04)])
        result=plan_insert(Model(),q,0.,q[:3]*1000,q[:3]*1000+160.*axis,np.eye(3),np.eye(4),[[-180.,180.]]*6,
                           fixed_z_axis=axis,fixed_height_up=up)
        points=np.asarray(result['tool_points_mm']);np.testing.assert_allclose(points@up,np.full(len(points),points[0]@up),atol=1e-10)
        with self.assertRaises(ValueError):plan_insert(Model(),q,0.,q[:3]*1000,q[:3]*1000+160.*axis,np.eye(3),np.eye(4),[[-180.,180.]]*6,fixed_z_axis=axis)

    def test_measured_approach_and_full_insertion_share_one_checked_path(self):
        from agent.held_insertion_path import plan_insert
        class Model:
            reach=.7
            def wrist_transform(self,q,h):
                result=np.eye(4);result[:3,3]=q[:3];return result
            def inverse_wrist(self,q,h,xyz,rpy,limits):
                result=np.array(q);result[:3]=np.asarray(xyz)/1000;return result
            def check(self,a,b,h):return {'clear':True,'lower_mm':100.}
        axis=np.array([1.,0.,0.])
        args=(Model(),np.zeros(6),0.,np.zeros(3),axis*273.,np.eye(3),np.eye(4),[[-180.,180.]]*6)
        plan=plan_insert(*args,fixed_z_axis=axis,measured_gap_mm=123.)
        x=np.asarray(plan['tool_points_mm'])[:,0]
        self.assertIn(123.,x)
        self.assertAlmostEqual(x[-1],273.)
        self.assertLessEqual(np.diff(x).max(),2.000001)
        self.assertTrue((np.diff(x)>0).all())
        for gap in [None,0.,12.,202.,float('nan')]:
            with self.assertRaises(ValueError):plan_insert(*args,fixed_z_axis=axis,measured_gap_mm=gap)
        with self.assertRaises(ValueError):
            plan_insert(*args[:4],axis*301.,*args[5:],fixed_z_axis=axis,measured_gap_mm=151.)

if __name__=='__main__':unittest.main()
