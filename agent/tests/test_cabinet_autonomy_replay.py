import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from agent.cabinet_replay import run_demo,replay_recorded
from agent.tests.test_held_rgb import fixture
from agent.tests.test_cabinet_rgb_geometry import rgb_frame

ROOT=Path(__file__).resolve().parents[2]


class ReplayTests(unittest.TestCase):
    def test_full_simulated_flow_connects_real_rgb_matcher_and_gates(self):
        with tempfile.TemporaryDirectory() as t,patch.object(socket,'socket',side_effect=AssertionError('Network forbidden')):
            r=run_demo(Path(t))
            success=r['scenarios']['success'];self.assertEqual(success['state']['status'],'completed')
            self.assertEqual([c['action'] for c in success['commands']],['base_align','lift_align','outside_align','insert','release','withdraw_empty'])
            self.assertFalse(r['executable']);self.assertEqual(r['mode'],'offline')
            self.assertTrue((Path(t)/'index.html').is_file());self.assertTrue((Path(t)/'report.json').is_file())
    def test_slip_occlusion_precision_and_manual_cases_do_not_fake_success(self):
        with tempfile.TemporaryDirectory() as t:
            r=run_demo(Path(t))
            for name in ('slip','occlusion','manual'):
                self.assertIn(r['scenarios'][name]['state']['status'],('stopping','recovery_required'))
                self.assertTrue(r['scenarios'][name]['stopping'])
            precision=r['scenarios']['precision'];self.assertIn('G12',precision['last']['decision']['blocked'])
            self.assertNotIn('insert',[c['action'] for c in precision['commands']])
    def test_recorded_unreferenced_d405_is_unknown_not_qualified(self):
        with tempfile.TemporaryDirectory() as t,patch.object(socket,'socket',side_effect=AssertionError('Network forbidden')):
            p=Path(t);cv2.imwrite(str(p/'d405.jpg'),fixture()[0][:,:,::-1]);f=rgb_frame();np.savez_compressed(p/'d435.npz',**f)
            r=replay_recorded(p/'out',d435_frame=p/'d435.npz',d405_image=p/'d405.jpg')
            self.assertEqual(r['held']['holding_eligibility'],'unknown');self.assertIn('G09',r['decision']['blocked']);self.assertTrue(r['historical'])
            self.assertFalse(r['executable']);self.assertFalse(r['physical_accuracy_verified'])
    def test_cli_runs_offline_demo_without_project_cwd(self):
        with tempfile.TemporaryDirectory() as t:
            p=subprocess.run([sys.executable,str(ROOT/'scripts/replay_cabinet_autonomy.py'),'--demo','--output',str(Path(t)/'report')],cwd=t,capture_output=True,text=True)
            self.assertEqual(p.returncode,0,p.stderr);self.assertEqual(json.loads(p.stdout)['mode'],'offline')
    def test_reference_metadata_cannot_be_inferred_from_arbitrary_current_image(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);cv2.imwrite(str(p/'frame.jpg'),fixture()[0][:,:,::-1])
            r=replay_recorded(p/'out',d405_image=p/'frame.jpg');self.assertEqual(r['held']['reason'],'Qualified D405 reference missing')
