import hashlib
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from d405_left_edges import CANDIDATE, measure_left_edges


class GapTests(unittest.TestCase):
    def test_deadband_and_direction_in_base_y(self):
        root=Path(__file__).resolve().parents[2]
        f=dict(id='frame',image_time=1.,arm_time=1.,body_time=1.,camera_id='d405',
               calibration_sha256=hashlib.sha256(CANDIDATE.read_bytes()).hexdigest(),
               rgb=np.zeros((3,4,3),np.uint8),xyz=np.zeros((3,4,3)),base_from_color=np.eye(4),
               intrinsics='{}')
        rim=np.zeros((32,3));rim[:,2]=np.linspace(.48,.51,32)
        pixels=np.zeros((32,2),int)
        for gap,step,inside in [(-2.,-4.5,False),(0.,0.,True),(2.,0.,True),(5.,0.,True),(13.,10.5,False),
                                (32.9,30.4,False),(-27.9,-30.4,False)]:
            battery=rim.copy();battery[:,1]=-gap/1000
            with patch('d405_left_edges.camera_intrinsics',return_value=(None,None)), \
                 patch('d405_left_edges._inner_left',return_value=(rim,pixels,0.,0.,.49,np.array([1.,0.,0.]))), \
                 patch('d405_left_edges._left_battery',return_value=(battery,pixels)):
                result=measure_left_edges(f,np.eye(4),root)
            self.assertEqual(result['in_band'],inside)
            self.assertAlmostEqual(result['step_mm'],step)
            self.assertAlmostEqual(result['clearance_mm'],gap)

    def test_fractional_travel_and_pose_at_exposure(self):
        from d405_left_edges import lateral_step_mm
        origin=.18
        for sign in (-1,1):
            with self.subTest(sign=sign):
                # Only 0.018 mm remains available on this side of the taught corridor.
                boundary=origin+sign*.015
                self.assertEqual(lateral_step_mm(sign*5.,boundary,boundary,origin,15.),0.)
                near=origin+sign*.014982
                step=lateral_step_mm(sign*5.,near,near,origin,15.)
                self.assertAlmostEqual(step,sign*.018,places=8)
                self.assertAlmostEqual(lateral_step_mm(sign*12.7,origin,origin+sign*.002,origin,15.),
                                       sign*10.7,places=8)

    def test_full_measured_target_accounts_for_motion_since_exposure(self):
        from d405_left_edges import lateral_step_mm
        for sign in (-1,1):
            with self.subTest(sign=sign):
                self.assertAlmostEqual(lateral_step_mm(sign*30.4,.18,.18+sign*.002,.18),sign*28.4)
                self.assertAlmostEqual(lateral_step_mm(sign*10.,.18+sign*.025,.18+sign*.025,.18),sign*10.)

    def test_side_plane_recovers_y_with_hidden_nose_and_rejects_missing_support(self):
        from d405_left_edges import _side_at_mouth
        from agent.docking_fit import OBJECT_TO_TOOL
        # Camera axes in this synthetic point cloud are the base axes. The tool
        # origin anchors the battery, whose observed left face is 3 mm inside the lip.
        rim=np.zeros((60,3));rim[:,0]=.60;rim[:,1]=.020;rim[:,2]=np.linspace(.46,.52,60)
        tool=np.eye(4);tool[:3,3]=[.441,-.019,.49]
        bounds=np.array([[-46.,-37.5,0.],[36.3,37.5,195.]])
        contact=np.array([0.,0.,169.])
        self.assertGreater((tool[:3,:3]@OBJECT_TO_TOOL)[1,0],.85)
        distance,height=np.meshgrid(np.linspace(.009,.075,70),np.linspace(.467,.515,48))
        xyz=np.stack((.60-distance,.017+.05*distance,height),axis=-1)
        rgb=np.full(xyz.shape,72,np.uint8)
        gaps,points,pixels=_side_at_mouth(rgb,xyz,np.eye(4),tool,bounds,contact,rim,np.array([1.,0.,0.]),0.,.020,.49)
        self.assertAlmostEqual(np.median(gaps),3.,places=4)
        self.assertEqual(len(points),len(pixels))
        translated=xyz.copy();translated[:,:,1]+=.004
        gaps,_,_=_side_at_mouth(rgb,translated,np.eye(4),tool,bounds,contact,rim,np.array([1.,0.,0.]),0.,.020,.49)
        self.assertAlmostEqual(np.median(gaps),-1.,places=4)
        for bad in (xyz*0.,xyz+np.array([.15,0.,0.]),np.where((distance>.04)[:,:,None],xyz,0.)):
            with self.assertRaises(ValueError):
                _side_at_mouth(rgb,bad,np.eye(4),tool,bounds,contact,rim,np.array([1.,0.,0.]),0.,.020,.49)

    def test_stage_can_complete_y_without_height_or_other_rim_edges(self):
        from test_placement_stages import StageTests
        import time
        case=StageTests();case.setUp();self.addCleanup(case.doCleanups)
        settings=case.task.snapshot()['settings'];settings['correction_axes']='y'
        case.task.action(dict(op='save_settings',settings=settings))
        (case.root/'config/battery-execution.json').write_bytes((Path(__file__).resolve().parents[2]/'config/battery-execution.json').read_bytes())
        f={'id':'same-exposure','image_time':time.time()}
        observed=dict(image_time=f['image_time'],in_band=True,raw_error_mm=0.,step_mm=0.,clearance_mm=4.)
        with patch.object(case.task,'_left_only',return_value=(f,None)),patch.object(case.task,'_left_measurement',return_value=observed), \
             patch.object(case.task,'_visual_height',side_effect=ValueError('视觉高度不可用')), \
             patch.object(case.task,'_capture',side_effect=AssertionError('unexpected height capture')):
            case.task.action(dict(op='run',phase='correct',expected_epoch=10,request_id='only-y'))
            case.task.thread.join(4.)
        self.assertEqual(case.task.state['status'],'completed',case.task.state.get('reason'))
        self.assertEqual(case.task.state['d405_left_clearance_mm'],4.)
        self.assertFalse(case.task.state['trajectory_handoffs'])

    def test_out_of_band_target_only_translates_in_base_y(self):
        from test_placement_stages import StageTests
        import time
        case=StageTests();case.setUp();self.addCleanup(case.doCleanups);case.ctrl.target=0.
        s=case.task.snapshot()['settings'];s['correction_axes']='y'
        case.task.action(dict(op='save_settings',settings=s))
        (case.root/'config/battery-execution.json').write_bytes((Path(__file__).resolve().parents[2]/'config/battery-execution.json').read_bytes())
        before=case.task._tool(case.ctrl.snapshot())
        f=dict(id='test',image_time=time.time()); poses=[]
        def capture_plan(model,q,lift,tcp,target,limits,**kw):
            poses.append(target.copy())
            raise ValueError('plan only')
        with patch.object(case.task,'_left_only',return_value=(f,None)),patch.object(case.task,'_left_measurement',
                return_value=dict(image_time=f['image_time'],in_band=False,raw_error_mm=30.4,step_mm=30.4,clearance_mm=32.9,observed_y_m=before[1,3])), \
             patch('placement_stages.plan_correction',side_effect=capture_plan):
            case.task.action(dict(op='run',phase='correct',expected_epoch=10,request_id='positive-y'))
            case.task.thread.join(4.)
        expected=before.copy();expected[1,3]+=.0304
        self.assertEqual(len(poses),1)
        np.testing.assert_allclose(poses[0],expected)


    def test_stage_submillimetre_remaining_target_reaches_planner(self):
        from test_placement_stages import StageTests
        import time
        for direction in (-1,1):
            with self.subTest(direction=direction):
                case=StageTests();case.setUp();self.addCleanup(case.doCleanups)
                settings=case.task.snapshot()['settings'];settings['correction_axes']='y'
                case.task.action(dict(op='save_settings',settings=settings))
                (case.root/'config/battery-execution.json').write_bytes(
                    (Path(__file__).resolve().parents[2]/'config/battery-execution.json').read_bytes())
                before=case.task._tool(case.ctrl.snapshot())
                frame=dict(id='fresh',image_time=time.time())
                # The gap at exposure requests 5 mm, of which 4.982 mm has already been travelled.
                observation=dict(image_time=frame['image_time'],in_band=False,raw_error_mm=direction*5.,
                                 clearance_mm=2.5+direction*5.,observed_y_m=before[1,3]-direction*.004982)
                targets=[]
                def plan(model,q,height,tcp,target,limits,**kw):
                    targets.append(target.copy())
                    raise ValueError('plan only')
                with patch.object(case.task,'_left_only',return_value=(frame,None)), \
                     patch.object(case.task,'_left_measurement',return_value=observation), \
                     patch('placement_stages.plan_correction',side_effect=plan):
                    case.task.action(dict(op='run',phase='correct',expected_epoch=10,request_id='small-y'))
                    case.task.thread.join(4.)
                self.assertEqual(len(targets),1)
                expected=before.copy();expected[1,3]+=direction*.000018
                np.testing.assert_allclose(targets[0],expected,atol=1.e-12)
