"""Offline regression for selected prompts and Plan F target association."""
import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch

import numpy as np
from PIL import Image
from agent.perception_worker import Models,VISION


class SelectedPerceptionProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        Image.new('RGB',(80,60)).save(self.root/'rgb.png')
        np.savez(self.root/'frame.npz',camera_id='d405')
        self.target=types.ModuleType('target')
        self.target.shape_view=lambda image:image
        self.target.load_reference=lambda:Image.new('RGB',(80,60))
        self.target.DINO_PROMPT='default handle';self.target.TASK='default task'
        spec=importlib.util.spec_from_file_location('selected_profile_pipeline',VISION/'pipeline.py')
        self.pipeline=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,target=self.target):spec.loader.exec_module(self.pipeline)
        self.judge=SimpleNamespace(records=[],select=Mock(return_value={'status':'selected','candidate_id':1}),
            review=Mock(side_effect=AssertionError('No extra Qwen mask review')))
        self.pipeline.QwenJudge=Mock(return_value=self.judge)
        self.model_module=types.ModuleType('models');self.model_module.VisionModels=Mock()
        self.row={'candidate_id':1,'bbox_xyxy':[8,8,32,32],'score':.8,'label':'battery'}
        self.mask=np.zeros((60,80),bool);self.mask[10:30,10:30]=True
        self.worker=Models();self.worker.vision=Mock()
        self.worker.vision.detect.return_value=[self.row]
        self.worker.vision.segment.return_value=(self.mask,.95)

    def segment(self,name,chosen=None,location=None,mode='plan_f',depth_error=None):
        with patch.dict(sys.modules,{'target':self.target,'pipeline':self.pipeline,'models':self.model_module}), \
                patch('agent.perception_worker.validate_frame'), \
                patch('agent.perception_worker.local_choice',return_value=chosen) as local, \
                patch('agent.battery_observation.surface_location',return_value=location or {'surface_base_mm':[100.,20.,500.]},
                      side_effect=depth_error):
            result=self.worker.segment({'frame':str(self.root/'frame.npz'),'image':str(self.root/'rgb.png'),
                'output':str(self.root/name),'vision_mode':mode,'dino_prompt':'selected battery',
                'qwen_task':'selected battery task','recognition_profile_id':'battery'})
        return result,local

    def test_qwen_fallback_and_retry_use_selected_dino_and_qwen_prompts(self):
        for name in ('first','retry'):
            result,_=self.segment(name)
            self.assertEqual(result['path'],'qwen_fallback')
            self.assertTrue(all(call.args[1]=='selected battery' for call in self.worker.vision.detect.call_args_list))
            self.assertEqual(self.judge.select.call_args.args[3],'selected battery task')
            self.judge.review.assert_not_called()

    def test_first_d405_target_is_reused_and_not_replaced_by_later_candidates(self):
        chosen=(self.row,self.mask,.95)
        self.segment('first',chosen=chosen)
        np.testing.assert_array_equal(self.worker.head_target,[100.,20.,500.])
        _,local=self.segment('next',chosen=chosen,location={'surface_base_mm':[700.,0.,500.]})
        np.testing.assert_array_equal(local.call_args.args[4],[100.,20.,500.])
        np.testing.assert_array_equal(self.worker.head_target,[100.,20.,500.])

    def test_invalid_initial_depth_leaves_target_unset_for_existing_retry(self):
        chosen=(self.row,self.mask,.95)
        self.segment('invalid',chosen=chosen,depth_error=ValueError('Invalid depth'))
        self.assertIsNone(self.worker.head_target)
        self.segment('retry',chosen=chosen)
        np.testing.assert_array_equal(self.worker.head_target,[100.,20.,500.])

    def test_plan_a_d405_does_not_replace_missing_d435_association(self):
        self.segment('plan-a',chosen=(self.row,self.mask,.95),mode='d405')
        self.assertIsNone(self.worker.head_target)

    def test_task_profile_and_mode_are_forwarded_to_worker(self):
        from agent.grasp_pipeline import segment
        store=Mock();store.path=self.root
        store.state={'battery_task_settings':{'vision_mode':'plan_f','recognition_profile':{
            'id':'battery','dino_prompt':'selected battery','qwen_task':'selected battery task'}}}
        store.observation.return_value={'camera_id':'d405'}
        store.image_path.return_value=self.root/'rgb.png'
        store.perception_session.call.return_value={'status':'needs_new_observation','output':str(self.root/'source')}
        output=self.root/'task-output';output.mkdir()
        with patch('agent.grasp_pipeline.frame_path',return_value=self.root/'frame.npz'):
            segment(store,'observation',output,lambda:False)
        arguments=store.perception_session.call.call_args.args[1]
        self.assertEqual(arguments['vision_mode'],'plan_f')
        self.assertEqual(arguments['dino_prompt'],'selected battery')
        self.assertEqual(arguments['qwen_task'],'selected battery task')


if __name__=='__main__':unittest.main()
