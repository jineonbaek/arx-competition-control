"""Local selection and task cleanup without loading CUDA or moving hardware."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
from types import SimpleNamespace
import json
import numpy as np
from agent.perception_session import PerceptionSession
from agent.perception_worker import local_choice,projected_box
from agent.battery_flow import BatteryFlow
from agent.storage import RunStore


class PerceptionTests(unittest.TestCase):
    def test_duplicate_boxes_do_not_force_cloud_and_distinct_ambiguous_boxes_do(self):
        rows=[dict(candidate_id=1,bbox_xyxy=[0,0,10,10],score=.38),
              dict(candidate_id=2,bbox_xyxy=[0,0,11,10],score=.35)]
        models=Mock();models.segment.return_value=(np.ones((20,20),bool),.95)
        import sys,types
        target=types.ModuleType('target');target.shape_view=lambda image:image
        with patch.dict(sys.modules,target=target),patch('agent.battery_observation.surface_location',
                return_value={'surface_base_mm':[0.,0.,500.]}):
            self.assertEqual(local_choice(rows,{},models,None)[0]['candidate_id'],1)
            rows[1]['bbox_xyxy']=[10,10,20,20]
            self.assertIsNone(local_choice(rows,{},models,None))
            self.assertIsNone(local_choice(rows[:1],{},models,None,np.array([1000,0,500.])))

    def test_tracking_allows_five_percent_overflow_without_changing_other_checks(self):
        import sys,types
        from PIL import Image
        from agent.perception_worker import Models
        target=types.ModuleType('target')
        target.shape_view=lambda image:image
        target.load_reference=lambda:None
        target.DINO_PROMPT='fixture';target.TASK='fixture'
        pipeline=types.ModuleType('pipeline')
        pipeline.QwenJudge=Mock(return_value=SimpleNamespace(records=[]))
        pipeline.run_frame=Mock(side_effect=lambda *a,**kw:{'status':'perception_accepted'})
        pipeline.overlay_mask=lambda image,mask:image
        pipeline.filter_fixed_gripper_candidates=lambda rows,size,camera:(rows,[])
        pipeline.mask_geometry=lambda mask:{}
        model_module=types.ModuleType('models');model_module.VisionModels=Mock()
        for outside,score,depth_ok,tracked in [(0,.95,True,True),(4,.95,True,True),
                (5,.95,True,True),(6,.95,True,False),
                (0,.84,True,False),(0,.95,False,False)]:
            with self.subTest(outside=outside,score=score,depth_ok=depth_ok), tempfile.TemporaryDirectory() as root:
                root=Path(root);Image.new('RGB',(14,14)).save(root/'rgb.png')
                np.savez(root/'frame.npz',camera_id='d405')
                mask=np.zeros((14,14),bool)
                inside_y,inside_x=np.indices((10,10))
                mask[inside_y.ravel()[:100-outside]+2,inside_x.ravel()[:100-outside]+2]=True
                mask[2:2+outside,12]=True
                worker=Models();worker.pose={'base_from_object':np.eye(4)}
                worker.vision=Mock();worker.vision.segment.return_value=(mask,score)
                worker.vision.detect.return_value=[]
                with patch.dict(sys.modules,{'target':target,'pipeline':pipeline,'models':model_module}), \
                        patch('agent.perception_worker.validate_frame'), \
                        patch('agent.perception_worker.projected_box',return_value=[2,2,12,12]), \
                        patch('agent.perception_worker.local_choice',return_value=None), \
                        patch('agent.battery_observation.surface_location',
                              side_effect=None if depth_ok else ValueError('Invalid depth')):
                    report=worker.segment({'frame':str(root/'frame.npz'),
                        'image':str(root/'rgb.png'),'output':str(root/'out'),
                        'dino_prompt':'selected dino','qwen_task':'selected qwen',
                        'recognition_profile_id':'selected-profile'})
                self.assertEqual(report['path'],'pose_tracking' if tracked else 'qwen_fallback')
                self.assertEqual(worker.vision.detect.called,not tracked)
                self.assertEqual(report['recognition_profile_id'],'selected-profile')
                if not tracked:
                    self.assertEqual(worker.vision.detect.call_args.args[1],'selected dino')
                    self.assertEqual(pipeline.run_frame.call_args.args[4],'selected qwen')

    def test_real_vision_fallback_selects_without_qwen_mask_review(self):
        import importlib.util,sys,types
        from PIL import Image
        from agent.perception_worker import Models,VISION
        target=types.ModuleType('target')
        target.shape_view=lambda image:image
        target.load_reference=lambda:Image.new('RGB',(80,60))
        target.DINO_PROMPT='fixture';target.TASK='fixture'
        spec=importlib.util.spec_from_file_location('candidate_only_pipeline',VISION/'pipeline.py')
        pipeline=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,target=target):spec.loader.exec_module(pipeline)
        judge=SimpleNamespace(records=[],select=Mock(return_value={'status':'selected','candidate_id':1}),
            review=Mock(side_effect=AssertionError('Qwen mask review must not run')))
        pipeline.QwenJudge=Mock(return_value=judge)
        model_module=types.ModuleType('models');model_module.VisionModels=Mock()
        row=dict(candidate_id=1,bbox_xyxy=[8,8,32,32],score=.8,label='battery')
        mask=np.zeros((60,80),bool);mask[10:30,10:30]=True
        for ambiguous in (False,True):
            with self.subTest(ambiguous=ambiguous),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);Image.new('RGB',(80,60)).save(root/'rgb.png')
                np.savez(root/'frame.npz',camera_id='d405')
                worker=Models();worker.vision=Mock()
                worker.vision.detect.return_value=[row];worker.vision.segment.return_value=(mask,.95)
                chosen=None if ambiguous else (row,mask,.95)
                with patch.dict(sys.modules,{'target':target,'pipeline':pipeline,'models':model_module}), \
                        patch('agent.perception_worker.validate_frame'), \
                        patch('agent.perception_worker.local_choice',return_value=chosen):
                    result=worker.segment(dict(frame=str(root/'frame.npz'),image=str(root/'rgb.png'),output=str(root/'out')))
                self.assertEqual(result['status'],'perception_accepted')
                self.assertEqual(result['path'],'qwen_fallback' if ambiguous else 'local_candidate')
                if ambiguous:
                    judge.select.assert_called_once()
                    self.assertEqual(result['review']['status'],'skipped')
                else:judge.select.assert_not_called()
                judge.review.assert_not_called()

    def test_qwen_precedes_codex_region_even_with_local_recovery_enabled(self):
        import sys,types
        from PIL import Image
        from agent.perception_worker import Models
        row=dict(candidate_id=1,bbox_xyxy=[2,3,20,30],score=.8)
        mask=np.zeros((100,100),bool);mask[21:40,30:50]=True
        for local,qwen_ok,region in ((True,False,True),(False,True,True),
                                     (False,False,True),(False,False,False)):
            with self.subTest(local=local,qwen_ok=qwen_ok,region=region), tempfile.TemporaryDirectory() as td:
                root=Path(td);Image.new('RGB',(100,100)).save(root/'rgb.jpg')
                np.savez(root/'frame.npz',camera_id='d405')
                calls=[]
                target=types.ModuleType('target');target.shape_view=lambda image:image
                target.load_reference=lambda:None;target.DINO_PROMPT=target.TASK='fixture'
                pipeline=types.ModuleType('pipeline')
                pipeline.QwenJudge=Mock(return_value=SimpleNamespace(records=[]))
                def qwen(*args,**kwargs):
                    calls.append('qwen')
                    return dict(status='perception_accepted' if qwen_ok else 'needs_new_observation',
                                selection=dict(status='selected' if qwen_ok else 'not_found'))
                pipeline.run_frame=Mock(side_effect=qwen)
                pipeline.filter_fixed_gripper_candidates=lambda rows,*args:(rows,[])
                pipeline.overlay_mask=lambda image,mask:image;pipeline.mask_geometry=lambda mask:dict(mask_valid=True)
                models=types.ModuleType('models');models.VisionModels=Mock()
                worker=Models();worker.vision=Mock()
                def detect(image,prompt):
                    calls.append('detect');return [row] if local or image.size!=(100,100) else []
                worker.vision.detect.side_effect=detect
                def choose(rows,*args):return (rows[0],mask,.91) if rows else None
                params=dict(frame=str(root/'frame.npz'),image=str(root/'rgb.jpg'),output=str(root/'out'),
                            local_recovery=True,dino_prompt='chosen prompt',qwen_task='chosen task')
                if region:params['search_region']=[400,300,800,700]
                with patch.dict(sys.modules,dict(target=target,pipeline=pipeline,models=models)), \
                     patch('agent.perception_worker.validate_frame'), \
                     patch('agent.perception_worker.local_choice',side_effect=choose):
                    report=worker.segment(params)
                expected='local_candidate' if local else ('qwen_fallback' if qwen_ok or not region else 'codex_region_local_detection')
                self.assertEqual(report['path'],expected)
                self.assertEqual(calls,['detect'] if local else (['detect','qwen'] if qwen_ok or not region else ['detect','qwen','detect']))
                if not local:
                    self.assertEqual(pipeline.run_frame.call_args.args[4],'chosen task')
                    self.assertFalse(pipeline.run_frame.call_args.kwargs['review_mask'])

    def test_session_is_lazy_and_close_releases_the_task_child(self):
        with tempfile.TemporaryDirectory() as root:
            with patch('agent.perception_session.subprocess.Popen') as launch:
                session=PerceptionSession(Path(root));self.assertFalse(launch.called)
                child=Mock(pid=12345);child.poll.return_value=None;session.child=child
                with patch('agent.perception_session.os.killpg') as terminate:
                    session.close();terminate.assert_called_once()
                child.wait.assert_called_once_with(timeout=3)
                self.assertIsNone(session.child)

    def test_warmup_loads_all_local_models_without_registering_a_scene(self):
        import sys,types
        from agent.perception_worker import Models
        vision=types.ModuleType('models');vision.VisionModels=Mock
        prepare=Mock()
        fp=types.ModuleType('foundationpose.infer_snapshot');fp.prepare_estimator=prepare
        with tempfile.TemporaryDirectory() as root,patch.dict(sys.modules,{
                'models':vision,'foundationpose.infer_snapshot':fp}):
            worker=Models();result=worker.warmup({'output':str(Path(root)/'warmup')})
        self.assertEqual(result['status'],'ready')
        self.assertIsInstance(worker.vision,Mock)
        prepare.assert_called_once()

    def test_whole_battery_and_nested_part_use_one_local_target(self):
        import sys,types
        target=types.ModuleType('target');target.shape_view=lambda image:image
        rows=[dict(candidate_id=4,bbox_xyxy=[2,2,8,6],score=.8),
              dict(candidate_id=2,bbox_xyxy=[1,1,9,9],score=.718)]
        whole=np.zeros((12,12),bool);whole[1:9,1:9]=True
        part=np.zeros_like(whole);part[2:6,2:8]=True
        models=Mock();models.segment.side_effect=[(part,.949),(whole,.970)]
        with patch.dict(sys.modules,target=target),patch('agent.battery_observation.surface_location',
                side_effect=[{'surface_base_mm':[620.,-140.,740.]},
                             {'surface_base_mm':[605.,-143.,720.]}]):
            chosen=local_choice(rows,{},models,None,np.array([632.,-132.,698.]))
        self.assertIsNotNone(chosen)
        self.assertEqual(chosen[0]['candidate_id'],2)
        np.testing.assert_array_equal(chosen[1],whole)

    def test_overlapping_boxes_with_distinct_masks_still_require_qwen(self):
        import sys,types
        target=types.ModuleType('target');target.shape_view=lambda image:image
        rows=[dict(candidate_id=1,bbox_xyxy=[0,0,10,10],score=.8),
              dict(candidate_id=2,bbox_xyxy=[1,0,11,10],score=.75)]
        left=np.zeros((12,12),bool);left[:10,:5]=True
        right=np.zeros_like(left);right[:10,6:11]=True
        models=Mock();models.segment.side_effect=[(left,.97),(right,.96)]
        with patch.dict(sys.modules,target=target),patch('agent.battery_observation.surface_location',
                side_effect=[{'surface_base_mm':[0.,0.,500.]},{'surface_base_mm':[0.,20.,500.]}]):
            self.assertIsNone(local_choice(rows,{},models,None,np.array([0.,10.,500.])))

    def test_nested_targets_at_different_depths_are_not_merged(self):
        import sys,types
        target=types.ModuleType('target');target.shape_view=lambda image:image
        rows=[dict(candidate_id=1,bbox_xyxy=[0,0,10,10],score=.8),
              dict(candidate_id=2,bbox_xyxy=[2,2,8,6],score=.75)]
        whole=np.zeros((12,12),bool);whole[:10,:10]=True
        part=np.zeros_like(whole);part[2:6,2:8]=True
        models=Mock();models.segment.side_effect=[(whole,.97),(part,.96)]
        with patch.dict(sys.modules,target=target),patch('agent.battery_observation.surface_location',
                side_effect=[{'surface_base_mm':[0.,0.,500.]},{'surface_base_mm':[0.,0.,580.]}]):
            self.assertIsNone(local_choice(rows,{},models,None,np.array([0.,0.,540.])))

    def test_projection_compensates_camera_movement_and_rejects_behind_camera(self):
        camera=np.eye(4);camera[0,3]=.1
        target=np.eye(4);target[2,3]=1.
        state={'base_from_object':target,'mesh':SimpleNamespace(vertices=np.array([[-.1,-.1,0],[.1,.1,0]]))}
        rs=SimpleNamespace(rs2_project_point_to_pixel=lambda intr,p:[100+100*p[0]/p[2],100+100*p[1]/p[2]])
        frame={'intrinsics':json.dumps({'width':200,'height':200})}
        with patch('agent.perception_worker.validate_frame',return_value=camera),patch(
                'web.src.camera_geometry.camera_intrinsics',return_value=(rs,None)):
            self.assertEqual(projected_box(frame,state),[76,86,104,114])
            state['base_from_object'][2,3]=-1
            with self.assertRaises(ValueError):projected_box(frame,state)

    def test_flow_releases_models_even_when_execution_raises(self):
        with tempfile.TemporaryDirectory() as root:
            store=RunStore.create(root)
            try:
                store.perception_session=Mock()
                flow=BatteryFlow(SimpleNamespace(store=store),review_plan=lambda *args:None)
                with patch.object(flow,'_run',side_effect=RuntimeError('fixture')):
                    with self.assertRaises(RuntimeError):flow.run()
                store.perception_session.close.assert_called_once()
            finally:store.close()


if __name__=='__main__':unittest.main()
