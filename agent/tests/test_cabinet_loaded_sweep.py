import unittest,hashlib,struct
import numpy as np
from agent.cabinet_loaded_sweep import infer_table,check_outside_sweep
class Model:
    reach=.5
    def __init__(self,y):
        self.lift=np.eye(4);self.lift[1,3]=y;self.column=(np.array([-3.,-3.,-3.]),np.array([-2.,-2.,-2.]))
        self.parts=[]
        for _ in range(8):
            m=np.eye(4);m[0,3]=.02
            c=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
            self.parts.append((m,np.array([0.,0.,1.]),c))
    def wrist_transform(self,q,lift):
        from scipy.spatial.transform import Rotation
        t=self.lift.copy()
        for a,(m,axis,_) in zip(q,self.parts[:6]):
            t=t@m;t[:3,:3]=t[:3,:3]@Rotation.from_rotvec(axis*a).as_matrix()
        return t
class SweepTests(unittest.TestCase):
    def test_upward_recovery_only_improves_positive_start_gap_and_restores_normal_margin(self):
        from agent.cabinet_loaded_sweep import check_translation_sweep
        self.table.update(front_offset_m=-.1,top_m=-.047,bottom_m=-.55)
        args=(self.model,self.left,[0.]*6,0.,[0.]*6,self.tcp,self.battery,self.window,self.table)
        with self.assertRaises(ValueError):check_translation_sweep(*args,[0.,0.,.006])
        r=check_translation_sweep(*args,[0.,0.,.006],upward_recovery=True)
        self.assertTrue(r['recovery_only']);self.assertAlmostEqual(r['lower_mm']['table'],9.5)
        self.assertAlmostEqual(r['end_table_gap_mm'],15.5);self.assertEqual(r['required_after_mm'],10.)
        for delta in [[0.,0.,-.006],[.006,0.,0.],[0.,0.,.02]]:
            with self.assertRaises(ValueError):check_translation_sweep(*args,delta,upward_recovery=True)
    def test_vertical_translation_union_keeps_ten_mm_start_gap_and_proves_entire_lift(self):
        from agent.cabinet_loaded_sweep import check_translation_sweep
        self.table.update(front_offset_m=-.1,top_m=-.048,bottom_m=-.55)
        r=check_translation_sweep(self.model,self.left,[0.]*6,0.,[0.]*6,self.tcp,self.battery,self.window,self.table,[0.,0.,.006],base_moves=False)
        self.assertAlmostEqual(r['lower_mm']['table'],10.5)
        self.assertEqual(r['continuous_geometry'],'translation_union')
        self.assertEqual(r['required_mm']['table'],10.)
    def test_translation_union_rejects_sweeping_through_table_even_with_clear_endpoints(self):
        from agent.cabinet_loaded_sweep import check_translation_sweep
        self.table.update(front_offset_m=.3,top_m=.005,bottom_m=-.005)
        self.tcp[2,3]=-.02;self.battery['bounds_mm']=[[-40.,-1.,0.],[40.,1.,195.]]
        for z in [-.02,.02]:
            tcp=self.tcp.copy();tcp[2,3]=z
            check_outside_sweep(self.model,self.left,[0.]*6,0.,[[0.]*6,[0.]*6],tcp,self.battery,self.window,self.table)
        with self.assertRaises(ValueError):
            check_translation_sweep(self.model,self.left,[0.]*6,0.,[0.]*6,self.tcp,self.battery,self.window,self.table,[0.,0.,.04],base_moves=False)
    def test_finer_continuous_sampling_proves_near_margin_without_lowering_ten_mm(self):
        self.table.update(front_offset_m=-.1,top_m=-.048,bottom_m=-.55)
        points=[[0.]*6,[.01]*6]
        with self.assertRaises(ValueError):
            check_outside_sweep(self.model,self.left,[0.]*6,0.,points,self.tcp,self.battery,self.window,self.table)
        r=check_outside_sweep(self.model,self.left,[0.]*6,0.,points,self.tcp,self.battery,self.window,self.table,sample_bound_mm=.2)
        self.assertAlmostEqual(r['lower_mm']['table'],10.3)
        self.assertEqual(r['required_mm']['table'],10.)
        self.table['top_m']=-.0474
        with self.assertRaises(ValueError):
            check_outside_sweep(self.model,self.left,[0.]*6,0.,points,self.tcp,self.battery,self.window,self.table,sample_bound_mm=.2)
    def test_static_pose_does_not_deduct_nonexistent_motion_between_samples(self):
        self.table.update(front_offset_m=-.1,top_m=-.048,bottom_m=-.55)
        r=check_outside_sweep(self.model,self.left,[0.]*6,0.,[[0.]*6,[0.]*6],self.tcp,self.battery,self.window,self.table)
        self.assertAlmostEqual(r['lower_mm']['table'],10.5)
        self.assertEqual(r['between_sample_bound_mm'],0.)
    def setUp(self):
        self.model=Model(-1.);self.left=Model(1.);self.tcp=np.eye(4);self.tcp[0,3]=.3
        self.battery={'bounds_mm':[[-40.,-37.5,0.],[40.,37.5,195.]],'contact_mm':[0.,0.,169.]}
        self.window={'center_base_m':[1.,-1.,0.],'outward_normal_base':[-1.,0.,0.]}
        self.table={'status':'inferred','front_normal_base':[1.,0.,0.],'front_offset_m':1.,'top_m':-.5,'bottom_m':-.55}
    def run_check(self):return check_outside_sweep(self.model,self.left,[0.]*6,0.,[[0.]*6,[.01]*6],self.tcp,self.battery,self.window,self.table)
    def test_clear_sweep_is_explicitly_nominal_and_bounded_between_samples(self):
        r=self.run_check();self.assertTrue(r['clear']);self.assertTrue(r['nominal_only']);self.assertLessEqual(r['between_sample_bound_mm'],1.00001)
    def test_endpoint_standoff_does_not_approve_intermediate_crossing(self):
        self.window['center_base_m']=[.58,-1.,0.]
        with self.assertRaises(ValueError):self.run_check()
    def test_table_collision_is_not_ignored(self):
        self.table.update(front_offset_m=.5,top_m=.3,bottom_m=-.3)
        with self.assertRaises(ValueError):self.run_check()
    def test_base_collision_is_not_ignored(self):
        self.model.column=(np.array([.4,-1.1,-.1]),np.array([.7,-.9,.1]))
        with self.assertRaises(ValueError):self.run_check()
    def test_missing_table_fails_closed(self):
        self.table={'status':'unavailable'}
        with self.assertRaises(ValueError):self.run_check()
class TableTests(unittest.TestCase):
    def test_table_front_comes_from_color_edge_and_checked_stl(self):
        rgb=np.full((480,640,3),120,np.uint8);rgb[:180,50:590]=[210,190,160]
        record=np.zeros(1,dtype=[('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]);record['v'][0]=[[0,45,0],[1,45,0],[0,46,0]]
        raw=bytes(80)+struct.pack('<I',1)+record.tobytes()
        t=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,.9],[0,0,0,1.]])
        frame={'rgb':rgb,'base_from_color':t,'intrinsics':{'fx':500.,'fy':500.,'cx':320.,'cy':0.,'width':640,'height':480,'model':'distortion.none','coeffs':[0.]*5}}
        w={'inner_quad_px':[[200,70],[400,70],[400,140],[200,140]],'center_base_m':[1.2,0,.7],'up_axis_base':[0,0,1.],'outward_normal_base':[-1,0,0]}
        profile={'sha256':hashlib.sha256(raw).hexdigest(),'plane_axes':[0,1]};slot={'center_mm':[0,100]}
        out=infer_table(frame,w,profile,slot,raw);self.assertEqual(out['status'],'inferred');self.assertTrue(out['nominal_only']);self.assertGreater(out['front_offset_m'],.5)
        with self.assertRaises(ValueError):infer_table(frame,w,profile,slot,raw+b'x')
