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

    def test_session_is_lazy_and_close_releases_the_task_child(self):
        with tempfile.TemporaryDirectory() as root:
            with patch('agent.perception_session.subprocess.Popen') as launch:
                session=PerceptionSession(Path(root));self.assertFalse(launch.called)
                child=Mock(pid=12345);child.poll.return_value=None;session.child=child
                with patch('agent.perception_session.os.killpg') as terminate:
                    session.close();terminate.assert_called_once()
                child.wait.assert_called_once_with(timeout=3)
                self.assertIsNone(session.child)

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
