import unittest
from unittest.mock import patch, Mock
import numpy as np
from agent.codex_recovery import Mailbox, session, CURRENT
from d405_left_edges import measure_left_edges

class FullLinkTests(unittest.TestCase):
    def test_color_only_failure_requests_and_remeasures_hint(self):
        mail=Mailbox();mail.enabled=True
        ctrl=Mock();ctrl.snapshot.return_value=dict(command_epoch=1,chassis_active=False,
            arms={'right':dict(fresh=True,joints=[0.]*6)},body=dict(fresh=True,height=0.))
        hint=dict(action='region',battery=[100,100,600,600],rim=[200,100,700,500],seed_deg=[],reason='region')
        frame=dict(capture_mode='color_only',image_time=1.,joints=[0.]*6,lift_rad=0.,rgb=np.zeros((20,20,3),np.uint8),calibration_sha256='test')
        def measure(*a,**k):
            if 'regions' not in k:raise ValueError('invisible')
            self.assertEqual(k['regions'],hint)
            return {'step_mm':2}
        with session('test',ctrl,lambda:True,Mock(),mail),patch('d405_rgb_alignment.measure_rgb_left_edges',side_effect=measure):
            for t in range(3):
                frame['image_time']=t
                with self.assertRaises(ValueError):measure_left_edges(frame,None,None)
                if t==0:self.assertEqual(len(mail.jobs),1)
            job=mail.take();self.assertIsNotNone(job)
            self.assertTrue(mail.reply(job['id'],hint))
            result=measure_left_edges(frame,None,None)
            self.assertTrue(result['codex_region_used'])
            self.assertTrue(any(e['event']=='applied' for e in mail.snapshot()['events']))

    def test_mailbox_history_no_images_and_bounded_queue(self):
        mail=Mailbox();mail.enabled=True
        for i in range(100):mail.open('r','diagnostic',dict(reason=str(i),images=['private']))
        snapshot=mail.snapshot()
        self.assertLessEqual(len(mail.jobs),32)
        self.assertNotIn('private',str(snapshot))
        self.assertLessEqual(len(snapshot['events']),200)

    def test_offline_retry_is_finite_and_does_not_wait_for_model(self):
        from agent.recovery_watch import RetryWatch
        clock=[0.];watch=RetryWatch('r','alignment',max_attempts=3,clock=lambda:clock[0])
        with patch('agent.recovery_watch.report_fault') as report:
            watch.fail('hidden');watch.fail('hidden')
            with self.assertRaisesRegex(ValueError,'不再原地重复'):watch.fail('hidden')
            self.assertTrue(report.called)

    def test_progress_resets_only_when_residual_improves(self):
        from agent.recovery_watch import RetryWatch
        watch=RetryWatch('r','alignment',max_attempts=3)
        with patch('agent.recovery_watch.report_fault'):
            watch.progress(10);watch.progress(10);watch.progress(11)
            self.assertEqual(watch.attempts,2)
            watch.progress(8);self.assertEqual(watch.attempts,0)
            watch.progress(8);watch.progress(8)
            with self.assertRaises(ValueError):watch.progress(8)

    def test_real_color_region_is_refit_not_used_as_a_measurement(self):
        import sys
        from pathlib import Path
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'web/tests'))
        from test_d405_rgb_alignment import frame,scene,ROOT
        from d405_rgb_alignment import measure_rgb_left_edges
        hint=dict(action='region',battery=[326,166,761,379],rim=[370,70,420,250],seed_deg=[],reason='test')
        result=measure_rgb_left_edges(frame(scene()),np.eye(4),ROOT,regions=hint)
        self.assertEqual(result['measurement_source'],'rgb_only')
        self.assertGreater(len(result['rim_left_px']),10)
        with self.assertRaises(ValueError):
            measure_rgb_left_edges(frame(np.zeros((480,640,3),np.uint8)),np.eye(4),ROOT,regions=hint)

    def test_diagnostic_cannot_release_or_move(self):
        from agent.codex_recovery import valid
        answer=dict(action='abstain',battery=[],rim=[],seed_deg=[],reason='check camera')
        self.assertTrue(valid('diagnostic',answer))
        self.assertFalse(valid('diagnostic',answer|{'action':'release'}))
        self.assertFalse(valid('diagnostic',answer|{'action':'seed','seed_deg':[0]*6}))

    def test_diagnostic_deduplicated_and_no_model_dependency(self):
        mail=Mailbox();mail.enabled=True
        a=mail.diagnostic('r','navigation','offline')
        b=mail.diagnostic('r','navigation','offline')
        self.assertEqual(a,b);self.assertEqual(len(mail.jobs),1)

if __name__=='__main__':unittest.main()
