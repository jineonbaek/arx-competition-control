import copy
import json
import shutil
import sys
import tempfile
import time
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from cabinet_preview import CabinetPreview
from agent.tests.test_cabinet_rgb_geometry import rgb_frame

ROOT=Path(__file__).resolve().parents[2]


class StateOnly:
    models={};config={'arms':{}}
    def snapshot(self):return copy.deepcopy(self.state)
    def submit(self,*args,**kwargs):raise AssertionError('preview called motor transport')
    def reserve_motion(self,*args,**kwargs):raise AssertionError('preview reserved motion')


class ValidityTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);root=Path(self.tmp.name)
        for rel in ['config/cabinet-profiles.json','config/battery-envelope.json','web/public/cad/battery-grip.json']:
            p=root/rel;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/rel,p)
        self.root=root;self.c=StateOnly();self.p=CabinetPreview(self.c,None,captures={},root=root)
        self.f=rgb_frame();k=json.loads(self.f['intrinsics']);k['fy']=650*66*169.6/(141*79);self.f['intrinsics']=json.dumps(k)
        self.f.update(joints=np.zeros(6),lift_rad=2.,right_gripper=-1.,command_epoch=7)
        now=self.f['image_time'];self.c.state=dict(motion_reserved=False,chassis_active=False,command_epoch=7,
            arms={'right':dict(fresh=True,joints=[0.]*6,gripper=-1.,timestamp=now)},
            body=dict(fresh=True,height=2.,timestamp=now))
        self.assertIn('plan',self.p.analyze_frame('d435','demo',self.f))

    def assert_invalid(self,now=None):
        r=self.p.snapshot(now=self.f['image_time'] if now is None else now)['d435']
        self.assertEqual(r['status'],'stale');self.assertFalse(r['validity']['valid'])
        for key in ['plan','window','alignment','arm_path']:self.assertNotIn(key,r)
        self.assertFalse(r['executable']);self.assertIn('image',r)

    def test_same_state_keeps_current_preview(self):
        self.assertIn('plan',self.p.snapshot(now=self.f['image_time'])['d435'])

    def test_expired_and_future_frames_clear_coordinates(self):
        for delta in [31,-1]:
            self.assert_invalid(self.f['image_time']+delta)

    def test_changed_epoch_pose_lift_gripper_or_motion_clears_coordinates(self):
        initial=copy.deepcopy(self.c.state)
        for key in ['epoch','joints','lift','gripper','moving','reserved','feedback','nan']:
            with self.subTest(key=key):
                self.c.state=copy.deepcopy(initial);self.p.analyze_frame('d435','demo',self.f)
                if key=='epoch':self.c.state['command_epoch']+=1
                elif key=='joints':self.c.state['arms']['right']['joints'][0]=.02
                elif key=='lift':self.c.state['body']['height']+=.1
                elif key=='gripper':self.c.state['arms']['right']['gripper']+=.1
                elif key=='moving':self.c.state['chassis_active']=True
                elif key=='reserved':self.c.state['motion_reserved']=True
                elif key=='feedback':self.c.state['arms']['right']['timestamp']-=1
                elif key=='nan':self.c.state['arms']['right']['joints'][0]=float('nan')
                self.assert_invalid()

    def test_changed_model_bytes_invalidate_without_motor_access(self):
        with (self.root/'config/cabinet-profiles.json').open('a') as f:f.write('\n')
        self.assert_invalid()

    def test_invalid_binding_cannot_pass_via_nan_comparisons(self):
        self.f['joints'][0]=float('nan');self.p.analyze_frame('d435','demo',self.f)
        self.assert_invalid()

    def test_model_is_reloaded_before_new_analysis(self):
        p=self.root/'config/cabinet-profiles.json';data=json.loads(p.read_text());data['demo']['openings'][0]['size_mm'][0]=170.
        p.write_text(json.dumps(data));r=self.p.analyze_frame('d435','demo',self.f)
        self.assertEqual(r['window']['model_aperture_size_mm'][0],170.)

    def test_calibration_changed_since_capture_does_not_publish_old_transform(self):
        from cabinet_capture import CALIBRATIONS
        path=self.root/CALIBRATIONS['d435'];path.parent.mkdir(parents=True);path.write_text('{}')
        r=self.p.analyze_frame('d435','demo',self.f)
        self.assertEqual(r['status'],'stale');self.assertNotIn('plan',r);self.assertNotIn('window',r)

    def test_action_rechecks_state_after_exposure_before_returning_coordinates(self):
        outer=self
        class Capture:
            def capture(self):
                outer.c.state['command_epoch']+=1
                return outer.f
        self.p.captures={'d435':Capture()}
        result=self.p.action({'camera':'d435','model':'demo'})
        self.assertEqual(result['status'],'stale');self.assertNotIn('plan',result)

    def test_robot_mesh_edits_invalidate_cached_path(self):
        package=self.root/'web/public/models/lift2/diy-20261105';(package/'urdf').mkdir(parents=True);(package/'meshes').mkdir()
        (package/'urdf/lift2_with_sensors.urdf').write_text('<robot><link name="base_link"><visual><geometry><mesh filename="package://lift2/meshes/base_link.STL"/></geometry></visual></link></robot>')
        mesh=package/'meshes/base_link.STL';mesh.write_bytes(b'initial CAD fixture')
        self.p.analyze_frame('d435','demo',self.f);mesh.write_bytes(b'changed CAD fixture')
        self.assert_invalid()
