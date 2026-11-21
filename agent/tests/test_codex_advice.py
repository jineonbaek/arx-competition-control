import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from agent.codex_recovery import (
    CURRENT, TTL, ApproachClearanceError, Episode, Mailbox, alternate_ik, observation_route, session, valid,
)
from d405_left_edges import measure_left_edges


def answer(action='region', battery=None, rim=None, seed=None):
    return dict(action=action, battery=battery or [], rim=rim or [], seed_deg=seed or [], reason='replay')


class AdviceTests(unittest.TestCase):
    def setUp(self):
        self.clock = 0.
        self.mail = Mailbox(clock=lambda: self.clock)
        self.mail.enabled = True
        self.active = True
        self.controller = Mock()
        self.state = dict(command_epoch=7, chassis_active=False,
                          arms=dict(right=dict(fresh=True, joints=[0.] * 6)),
                          body=dict(fresh=True, height=0.))
        self.controller.snapshot.return_value = self.state
        context = session('run1', self.controller, lambda: self.active, Mock(), self.mail)
        context.__enter__()
        self.addCleanup(context.__exit__, None, None, None)
        self.ctx = CURRENT.get()
        self.binding = ('d405', 'calib')
        self.region = answer(battery=[10, 20, 410, 550], rim=[240, 30, 500, 610])

    def miss(self, stamp, kind='edges'):
        self.ctx.failed(kind, self.binding, stamp, lambda: dict(id=stamp))

    def pending(self):
        for stamp in range(3):
            self.miss(stamp)
        return self.mail.take()

    def test_three_distinct_frames_one_job_without_a_mac(self):
        for stamp in [1, 1, 2, 2]:
            self.miss(stamp)
        self.assertIsNone(self.mail.take())
        self.miss(3)
        for stamp in range(4, 20):
            self.miss(stamp)
            self.assertIsNone(self.ctx.suggestion('edges', self.binding))
        self.assertEqual(self.mail.take()['evidence']['id'], 3)
        self.assertIsNone(self.mail.take())

    def test_normal_recovery_or_cancel_discards_late_reply(self):
        for recovery in ('end', 'cancel'):
            with self.subTest(recovery=recovery):
                job = self.pending()
                if recovery == 'end':
                    self.ctx.end('edges')
                else:
                    self.active = False
                    self.assertIsNone(self.ctx.suggestion('edges', self.binding))
                self.assertFalse(self.mail.reply(job['id'], self.region))

    def test_changes_of_pose_epoch_calibration_invalidate_answer(self):
        for change in ('pose', 'epoch', 'calibration'):
            with self.subTest(change=change):
                job = self.pending()
                self.assertTrue(self.mail.reply(job['id'], self.region))
                binding = self.binding
                if change == 'pose':
                    self.state['arms']['right']['joints'][1] += .03
                elif change == 'epoch':
                    self.state['command_epoch'] += 1
                else:
                    binding = ('d405', 'new-calibration')
                self.assertIsNone(self.ctx.suggestion('edges', binding))
                self.ctx.end('edges')

    def test_offline_expiry_reopens_only_on_fresh_observation(self):
        old = self.pending()
        self.clock += TTL+1
        self.assertFalse(self.mail.reply(old['id'], self.region))
        self.miss(3)
        new = self.mail.take()
        self.assertNotEqual(old['id'], new['id'])
        self.assertEqual(new['evidence']['id'], 3)
        self.assertTrue(self.mail.reply(new['id'], self.region))
        self.assertEqual(self.ctx.suggestion('edges', self.binding), self.region)

    def test_matrix_and_motor_commands_never_match_reply_schema(self):
        self.assertFalse(valid('grasp', self.region))
        self.assertFalse(valid('edges', self.region | dict(release=True)))
        self.assertFalse(valid('edges', self.region | dict(rim=[0, 5, 2000, 20])))
        self.assertFalse(valid('ik', answer('seed', seed=[float('nan')] * 6)))
        self.assertFalse(valid('ik', answer('seed', seed=[26] * 6)))
        self.assertFalse(valid('route', answer('release')))

    def test_stale_or_missing_poses_do_not_trigger_model(self):
        self.state['chassis_active'] = True
        self.assertIsNone(self.pending())
        self.assertFalse(self.ctx.frame_matches(dict(joints=[0.] * 6, lift_rad=0.)))
        self.state['chassis_active'] = False
        self.assertFalse(self.ctx.frame_matches(dict(joints=[0.] * 5, lift_rad=0.)))
        self.assertFalse(self.ctx.frame_matches(dict(joints=[.1] * 6, lift_rad=0.)))

    def test_d405_uses_local_measurement_after_reply_and_rejects_stale_image(self):
        frame = dict(image_time=0., joints=[0.] * 6, lift_rad=0., rgb=np.zeros((10, 10, 3),np.uint8),
                     calibration_sha256='calib')
        def measure(*args, **kwargs):
            if 'regions' not in kwargs:
                raise ValueError('left edges not visible')
            self.assertEqual(kwargs['regions'], self.region)
            return dict(raw_error_mm=2.4), None, None
        with patch('d405_left_edges.observe_left_pair', side_effect=measure):
            for stamp in (1., 2., 3.):
                frame['image_time'] = stamp
                with self.assertRaisesRegex(ValueError, 'left edges'):
                    measure_left_edges(frame, None, None)
            self.assertTrue(self.mail.reply(self.mail.take()['id'], self.region))
            frame.update(image_time=4.,joints=[1.] * 6)
            with self.assertRaisesRegex(ValueError,'left edges'):
                measure_left_edges(frame, None, None)
            frame.update(image_time=5.,joints=[0.] * 6)
            result=measure_left_edges(frame,None,None)
            self.assertEqual(result,dict(raw_error_mm=2.4,codex_region_used=True))
            self.assertFalse(self.mail.jobs)

    def test_seed_is_only_used_by_local_solver(self):
        q=np.zeros(6)
        goal=[0.]*7
        seed=[1,2,3,4,5,6]
        def solve(initial):
            job=self.mail.take()
            if job:
                self.assertTrue(self.mail.reply(job['id'],answer('seed',seed=seed)))
            if np.allclose(initial,np.radians(seed)):
                return 'locally validated'
            raise ValueError('no solution')
        self.assertEqual(alternate_ik(solve,q,q-1,q+1,goal,self.ctx),'locally validated')
        self.assertFalse(self.mail.jobs)

    def test_ik_search_terminates_with_no_mac(self):
        solver=Mock(side_effect=ValueError('no solution'))
        with self.assertRaisesRegex(ValueError,'no solution'):
            alternate_ik(solver,np.zeros(6),-np.ones(6),np.ones(6),[0.]*7,self.ctx)
        self.assertLessEqual(solver.call_count,39)
        self.assertEqual(self.mail.take()['kind'],'ik')

    def test_free_approach_checks_both_legs_and_retains_goal(self):
        start=np.zeros(6)
        goal=np.ones(6)
        seen=[]
        def check(a,b):
            seen.append((a.copy(),b.copy()))
            job=self.mail.take()
            if job:
                self.assertTrue(self.mail.reply(job['id'],answer('wrist_first')))
            if np.array_equal(b,goal) and np.array_equal(a,start) or (
                    np.array_equal(a,start) and b[4] <= .5):
                raise ApproachClearanceError('collision')
            return 'clear'
        points,proofs=observation_route(start,goal,check,self.ctx)
        self.assertEqual(proofs,['clear','clear'])
        np.testing.assert_array_equal(points[0],start)
        np.testing.assert_array_equal(points[-1],goal)
        np.testing.assert_array_equal(seen[-1][0],points[1])
        self.assertFalse(self.mail.jobs)

    def test_driver_failure_never_triggers_route_replanning(self):
        checker=Mock(side_effect=ValueError('driver changed'))
        with self.assertRaisesRegex(ValueError,'driver changed'):
            observation_route(np.zeros(6),np.ones(6),checker,self.ctx)
        checker.assert_called_once()
        self.assertFalse(self.mail.jobs)

    def test_terminal_failure_with_no_recovery_never_calls_network(self):
        from competition_modules import QwenRecovery
        recovery=QwenRecovery(Mock())
        recovery.client=Mock()
        result=recovery.decide(dict(failure='grasp not confirmed'),[])
        self.assertEqual(result,dict(name='stop',reason='grasp not confirmed'))
        recovery.client.complete.assert_not_called()

    def test_grasp_ignores_old_region_advice_and_preserves_qwen_failure(self):
        import sys
        import tempfile
        import types
        from pathlib import Path
        from PIL import Image
        from agent.perception_worker import Models
        target=types.ModuleType('target')
        target.shape_view=lambda image:image
        target.load_reference=Mock(return_value=None)
        target.DINO_PROMPT=target.TASK='battery'
        pipeline=types.ModuleType('pipeline')
        pipeline.QwenJudge=Mock(return_value=Mock(records=[]))
        pipeline.run_frame=Mock(return_value=dict(status='needs_new_observation',selection=dict(status='not_found')))
        pipeline.filter_fixed_gripper_candidates=lambda rows,*args:(rows,[])
        pipeline.overlay_mask=lambda image,mask:image
        pipeline.mask_geometry=lambda mask:dict(mask_valid=True)
        models=types.ModuleType('models')
        models.VisionModels=Mock()
        row=dict(candidate_id=1,bbox_xyxy=[2,3,20,30],score=.8)
        with tempfile.TemporaryDirectory() as td, patch.dict(sys.modules,dict(target=target,pipeline=pipeline,models=models)), patch('agent.perception_worker.validate_frame'):
            root=Path(td)
            Image.new('RGB',(100,100)).save(root/'rgb.jpg')
            np.savez(root/'frame.npz',camera_id='d405')
            params=dict(frame=str(root/'frame.npz'),image=str(root/'rgb.jpg'),output=str(root/'out'),local_recovery=True)
            worker=Models()
            worker.vision=Mock()
            worker.vision.detect.side_effect=[[],[],[row]]
            mask=np.zeros((100,100),bool)
            mask[21:40,30:50]=True
            def choose(rows,*args):
                return (rows[0],mask,.91) if rows else None
            with patch('agent.perception_worker.local_choice',side_effect=choose):
                self.assertEqual(worker.segment(params)['path'],'qwen_fallback')
                result=worker.segment(params | dict(search_region=[400,300,800,700]))
            self.assertEqual(result['path'],'qwen_fallback')
            self.assertEqual(result['status'],'needs_new_observation')
            self.assertEqual(worker.vision.detect.call_count,2)
            self.assertEqual(pipeline.QwenJudge.call_count,2)
