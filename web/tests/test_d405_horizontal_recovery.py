"""Regression: lost RGB edges must cause bounded Y search, not endless capture."""
import contextlib
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import test_d405_horizontal_flow as flow
import test_placement_stages as stages
from d405_rgb_alignment import color_battery_face


def invisible():
    # Exercise the real detector's missing-feature error, not a string classifier.
    color_battery_face(np.zeros((480,640,3),np.uint8))


class RecoveryFlowTests(unittest.TestCase):
    def setup_flow(self,*,lost=2,offline=False,stale=False,unaligned=False):
        level=flow.D405HorizontalFlowTests().make_level()
        level.guard=lambda:None;level.cancel=threading.Event()
        model=level.ctrl.models['right'];origin=model.wrist_transform(level.ctrl.q,level.height)
        captures=[];targets=[]
        def capture(timeout,**kw):
            captures.append(level.ctrl.q.copy())
            if len(captures)>30:raise AssertionError('still capturing without a recovery limit')
            if offline:raise TimeoutError('camera offline')
            return None,dict(id=str(len(captures)),camera_id='d405',image_time=0. if stale else 100.+len(captures)*.1,
                joints=level.ctrl.q.copy(),lift_rad=level.height)
        def measure(frame,**kw):
            if len(captures)<=lost:invisible()
            return dict(status='visual_guidance_only',axis_base=[0.,1.,0.],frame_id=frame['id'],
                        image_time=frame['image_time'],raw_error_mm=1. if unaligned else 0.,in_band=not unaligned)
        def plan(_,m,q,h,tcp,target,*args,**kw):
            targets.append(target.copy());end=q.copy();end[0]=target[1,3]
            return dict(joint_waypoints_rad=[q.tolist(),end.tolist()])
        level.capture_pair=capture
        stack=contextlib.ExitStack();self.addCleanup(stack.close)
        stack.enter_context(patch('fast_visual_level.measure_left_edges',side_effect=measure))
        stack.enter_context(patch('fast_visual_level.continuous_correction',side_effect=plan))
        stack.enter_context(patch('fast_visual_level.write_horizontal_overlay'))
        stack.enter_context(patch('time.time',side_effect=lambda:100.+len(captures)*.1))
        return level,model,origin,captures,targets

    def test_two_missing_frames_search_y_then_resume_alignment(self):
        level,model,origin,captures,targets=self.setup_flow()
        result=level.auto_d405_horizontal_trim(model,np.eye(4),origin)
        self.assertTrue(result['converged'])
        self.assertEqual(len(targets),1,'missing edges should move to a new observation position')
        expected=origin.copy();expected[1,3]+=.005
        np.testing.assert_allclose(targets[0],expected)
        self.assertAlmostEqual(level.ctrl.q[0],.005)

    def test_permanent_occlusion_exhausts_bounded_search_and_keeps_grip(self):
        level,model,origin,captures,targets=self.setup_flow(lost=100)
        with self.assertRaisesRegex(ValueError,'水平探查.*未识别'):
            level.auto_d405_horizontal_trim(model,np.eye(4),origin)
        self.assertEqual(len(targets),6)
        self.assertLessEqual(len(captures),14)
        self.assertTrue(any(t[1,3]>origin[1,3] for t in targets))
        self.assertTrue(any(t[1,3]<origin[1,3] for t in targets))
        for t in targets:
            self.assertLessEqual(abs(t[1,3]-origin[1,3]),.01500001)
            np.testing.assert_allclose(t[[0,2,3]],origin[[0,2,3]])
            np.testing.assert_allclose(t[:3,:3],origin[:3,:3])

    def test_camera_offline_does_not_move_and_does_not_wait_forever(self):
        level,model,origin,captures,targets=self.setup_flow(offline=True)
        with self.assertRaisesRegex(ValueError,'有效.*画面'):
            level.auto_d405_horizontal_trim(model,np.eye(4),origin)
        self.assertEqual(targets,[]);self.assertLessEqual(len(captures),3)

    def test_stale_missing_feature_frames_never_authorize_search(self):
        level,model,origin,captures,targets=self.setup_flow(lost=100,stale=True)
        with self.assertRaisesRegex(ValueError,'有效.*画面'):
            level.auto_d405_horizontal_trim(model,np.eye(4),origin)
        self.assertEqual(targets,[]);self.assertLessEqual(len(captures),3)

    def test_nonconvergent_measurements_have_a_total_observation_budget(self):
        level,model,origin,captures,targets=self.setup_flow(lost=0,unaligned=True)
        with self.assertRaisesRegex(ValueError,'观测预算'):
            level.auto_d405_horizontal_trim(model,np.eye(4),origin)
        self.assertLessEqual(len(captures),24)


class IndependentStageRecoveryTests(unittest.TestCase):
    def test_standalone_y_correction_searches_then_completes_without_insertion(self):
        case=stages.StageTests();case.setUp();self.addCleanup(case.doCleanups)
        task=case.task;ctrl=case.ctrl;captures=[];motions=[]
        source=Path(__file__).resolve().parents[2]/'config/battery-execution.json'
        (case.root/'config/battery-execution.json').write_bytes(source.read_bytes())
        settings=task.snapshot()['settings'];settings['correction_axes']='y'
        task.action(dict(op='save_settings',settings=settings))
        def capture():
            captures.append(ctrl.q.copy())
            if len(captures)>12:raise AssertionError('standalone correction still loops')
            return dict(id=str(len(captures)),image_time=100.+len(captures)*.1),None
        def measure(frame,after):
            if len(captures)<=2:invisible()
            return dict(image_time=frame['image_time'],in_band=True,raw_error_mm=0.,
                        clearance_mm=2.5,observed_y_m=-.195)
        def plan(_,model,q,height,tcp,target,*a,**kw):
            end=q.copy();end[:3]+=(target[:3,3]-model.wrist_transform(q,height)[:3,3])*1000
            return dict(joint_waypoints_rad=[q.tolist(),end.tolist()],timing={'time_from_start_s':[0.,1.]})
        class Motion:
            active=None;handoffs=[]
            def __init__(self,*a,**kw):pass
            def arm(self,parts,*a,**kw):
                ctrl.q=list(parts[-1]['joint_waypoints_rad'][-1]);motions.append(ctrl.q.copy())
                return ctrl.snapshot()
            def hold(self):pass
        with patch.object(task,'_left_only',side_effect=capture), \
             patch.object(task,'_left_measurement',side_effect=measure), \
             patch('placement_stages.continuous_correction',side_effect=plan), \
             patch('placement_stages.DockingMotion',Motion), \
             patch('time.time',side_effect=lambda:100.+len(captures)*.1):
            task.action(dict(op='run',phase='correct',expected_epoch=10,request_id='search'))
            task.thread.join(5.)
        result=task.snapshot()
        self.assertEqual(result['status'],'completed',result.get('reason'))
        self.assertEqual(len(motions),1)
        self.assertAlmostEqual(ctrl.q[1],5.)
        self.assertEqual(ctrl.q[0],0.);self.assertEqual(ctrl.q[2],0.)
        self.assertEqual(ctrl.target,0);self.assertEqual(case.push.calls,[])
        self.assertFalse(result['control_retained'])


if __name__=='__main__':unittest.main()
