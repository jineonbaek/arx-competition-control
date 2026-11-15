import unittest,tempfile,shutil,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from cabinet_preview import CabinetPreview
from agent.tests.test_cabinet_preview import frame
from agent.tests.test_cabinet_rgb_geometry import rgb_frame
ROOT=Path(__file__).resolve().parents[2]
class NoMotion:
    models={};config={'arms':{}}
    def snapshot(self):return {'motion_reserved':False,'chassis_active':False}
    def submit(self,*args,**kwargs):raise AssertionError('preview must never submit')
    def reserve_motion(self,*args):raise AssertionError('preview must never reserve')
class CabinetPreviewTests(unittest.TestCase):
    def make(self):
        t=tempfile.TemporaryDirectory();self.addCleanup(t.cleanup);root=Path(t.name)
        for rel in ['config/cabinet-profiles.json','config/battery-envelope.json','web/public/cad/battery-grip.json']:
            p=root/rel;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/rel,p)
        return CabinetPreview(NoMotion(),None,captures={},root=root)
    def test_no_motion_even_when_planning_fails(self):
        p=self.make();f=rgb_frame();k=json.loads(f['intrinsics']);k['fy']=650*66*169.6/(141*79);f['intrinsics']=json.dumps(k)
        r=p.analyze_frame('d435','demo',f);self.assertEqual(r['status'],'localized');self.assertFalse(r['executable']);self.assertEqual(r['plan']['z_range_mm'],0);self.assertEqual(r['arm_path']['status'],'rejected')
        self.assertEqual(r['environment_evidence']['status'],'unavailable');self.assertNotIn('observed_corridor_points',r)
        self.assertEqual(r['window']['localization_source'],'rgb_cad_pnp');self.assertFalse(r['readiness']['execution_ready'])
    def test_execution_capture_can_skip_duplicate_preview_ik_without_skipping_geometry(self):
        p=self.make();f=rgb_frame();k=json.loads(f['intrinsics']);k['fy']=650*66*169.6/(141*79);f['intrinsics']=json.dumps(k)
        def forbidden(*args):raise AssertionError('Duplicate preview IK must not run')
        p._arm_preview=forbidden
        r=p.analyze_frame('d435','demo',f,include_arm_preview=False)
        self.assertEqual(r['status'],'localized');self.assertEqual(r['plan']['z_range_mm'],0)
        self.assertEqual(r['arm_path']['status'],'deferred');self.assertFalse(r['executable'])
    def test_partial_view_has_no_inferred_plane_or_route(self):
        p=self.make();f=frame();f['rgb']=np.roll(f['rgb'],-200,axis=1);f['camera_id']='d405'
        r=p.analyze_frame('d405','demo',f);self.assertEqual(r['status'],'partial');self.assertNotIn('plan',r)
    def test_cannot_inject_motion_or_coordinates_as_preview_arguments(self):
        p=self.make()
        with self.assertRaises(ValueError):p.action({'camera':'d435','model':'demo','execute':True})
    def test_camera_identity_cannot_be_relabelled(self):
        p=self.make();f=frame();f['camera_id']='d405'
        with self.assertRaises(ValueError):p.analyze_frame('d435','demo',f)
    def test_depth_failure_still_shows_measured_2d_frame(self):
        p=self.make();f=rgb_frame();f['xyz']=np.full(f['rgb'].shape,np.nan)
        r=p.analyze_frame('d435','demo',f);self.assertIn(r['status'],('localized','planning_rejected'));self.assertEqual(len(r['detected_quad_px']),4)
        self.assertEqual(r['window']['localization_source'],'rgb_cad_pnp')
    def test_corridor_uses_checked_envelope_not_a_hardcoded_center(self):
        p=self.make();f={'xyz':np.array([[[.65,0,.65]]]),'base_from_color':np.eye(4)}
        w={'center_base_m':[.6,0,.65],'outward_normal_base':[-1,0,0],'left_axis_base':[0,1,0],'up_axis_base':[0,0,1]}
        plan={'window_model_size_mm':[169.6,79],'leading_insertion_depth_mm':140,'aperture_envelope_samples_mm':[[-2,-1],[3,-1],[3,1],[-2,1]]}
        self.assertEqual(p._corridor(f,w,plan),1)

class SeededIKTests(unittest.TestCase):
    def test_alternate_numeric_seed_still_checks_actual_start_sweep(self):
        from cabinet_preview import SeededModel
        class Base:
            def inverse_wrist(self,q,*args):
                if np.allclose(q,0):raise ValueError('singular start')
                return [.2]*6
            def check(self,a,b,lift):return {'clear':not np.allclose(a,0)}
        with self.assertRaises(ValueError):SeededModel(Base()).inverse_wrist([0]*6,5.,[200,0,600],[0,0,0],[[-180,180]]*6)
    def test_alternate_seed_can_return_only_after_actual_sweep_passes(self):
        from cabinet_preview import SeededModel
        class Base:
            def inverse_wrist(self,q,*args):
                if np.allclose(q,0):raise ValueError('singular start')
                return [.2]*6
            def check(self,a,b,lift):return {'clear':True}
        np.testing.assert_allclose(SeededModel(Base()).inverse_wrist([0]*6,5.,[200,0,600],[0,0,0],[[-180,180]]*6),[.2]*6)
