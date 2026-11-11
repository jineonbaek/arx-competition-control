import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from agent.tests.test_grasp_integration import frame
from agent.grasp_scene import save_frame, frame_path
from agent.grasp_geometry import validate_frame
from agent.storage import RunStore
from agent.config import Config
from agent.grasp_tools import GraspTools
from agent.battery_observation import surface_location, observe


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.store=RunStore.create(self.tmp.name);self.addCleanup(self.store.close)
        self.tools=GraspTools(Config(),self.store)

    def head_frame(self):
        f=frame();f['camera_id']=np.array('d435')
        f['xyz'][:]=[.1,0,2.];f['raw_depth'][:]=2000
        f['base_from_color'][:3,3]=[.2,-.25,.5]
        return f

    def test_head_archive_keeps_camera_transform_and_range(self):
        f=self.head_frame();buf=io.BytesIO();np.savez(buf,**f)
        obs=save_frame(self.store,buf.getvalue(),source='test',camera='d435')
        self.assertEqual(set(obs['cameras']),{'d435'})
        self.assertEqual(obs['camera_id'],'d435')
        self.assertTrue(frame_path(self.store,obs['observation_id']).is_file())
        np.testing.assert_allclose(validate_frame(f),f['base_from_color'])
        with self.assertRaises(ValueError):save_frame(self.store,buf.getvalue(),source='test',camera='d405')

    def test_coarse_surface_rejects_hole_background_and_reports_spread(self):
        f=self.head_frame()
        f['xyz'][:10,:,2]=2.0;f['raw_depth'][:10]=2000
        f['xyz'][10:,:,2]=.6;f['raw_depth'][10:]=600
        mask=np.ones((20,30),np.uint8)
        # Equal near and far supports are ambiguous, rather than their midpoint.
        with self.assertRaisesRegex(ValueError,'深度层'):surface_location(f,mask)
        f['xyz'][:15,:,2]=.6;f['raw_depth'][:15]=600
        f['xyz'][15:,:,2]=2.;f['raw_depth'][15:]=2000
        result=surface_location(f,mask)
        np.testing.assert_allclose(result['surface_base_mm'],[300,-250,1100],atol=.01)
        self.assertEqual(result['kind'],'visible_surface_not_grip')
        self.assertFalse(result['precision_validated'])
        self.assertEqual(result['support_pixels'],450)
        np.testing.assert_allclose(result['surface_bounds_p90_base_mm'],[[300,-250,1100]]*2,atol=.01)

    def test_head_capture_requests_own_endpoint_and_verifies_calibration(self):
        from agent.grasp_scene import capture
        from agent.tests.test_transport import Server,response
        root=Path(self.tmp.name)/'owner'
        cal=root/'vision/handeye/results/d435-cad-349622074226.json'
        cal.parent.mkdir(parents=True);cal.write_text('{}')
        archive=root/'web/hardware/runtime/grasp-head/frame.npz'
        archive.parent.mkdir(parents=True)
        f=self.head_frame();f['calibration_sha256']=np.array(__import__('hashlib').sha256(cal.read_bytes()).hexdigest())
        np.savez(archive,**f)
        def respond(path,body):
            self.assertEqual(path,'/api/grasp-scene')
            self.assertEqual(json.loads(body),{'camera':'d435'})
            return response({'id':'a'*32})
        with Server(respond) as server,patch('agent.grasp_scene.ROOT',root):
            self.tools.config.console_url=server.url
            captured=capture(self.tools,'d435')
            self.assertEqual(captured['camera_id'],'d435')
            f['calibration_sha256']=np.array('wrong');np.savez(archive,**f)
            with self.assertRaisesRegex(ValueError,'calibration'):capture(self.tools,'d435')

    def test_empty_or_wrong_mask_cannot_locate(self):
        for m in (np.zeros((20,30)),np.ones((2,3)),np.full((20,30),np.nan)):
            with self.assertRaises(ValueError):surface_location(self.head_frame(),m)

    def test_observe_rejection_archives_reason_without_pose_or_motion(self):
        f=self.head_frame();buf=io.BytesIO();np.savez(buf,**f)
        obs=save_frame(self.store,buf.getvalue(),source='test',camera='d435')
        with patch('agent.battery_observation.capture',return_value=obs),patch(
            'agent.battery_observation.segment',return_value={'status':'needs_new_observation','coarse_selection':{'status':'not_found'}}):
            out=observe(self.tools,'d435','observe_head')
        self.assertEqual(out['status'],'target_not_found')
        self.assertIsNone(out['location'])
        self.assertEqual(out['camera'],'d435')
        self.assertTrue((self.store.path/out['result_file']).is_file())
        self.assertNotIn('execute_pick',{t['function']['name'] for t in self.tools.definitions()})

    def test_observe_accepted_uses_mask_from_same_archive(self):
        from PIL import Image
        f=self.head_frame();buf=io.BytesIO();np.savez(buf,**f)
        obs=save_frame(self.store,buf.getvalue(),source='test',camera='d435')
        def segment(store,ident,out,cancelled):
            self.assertEqual(ident,obs['observation_id'])
            Image.fromarray(np.full((20,30),255,np.uint8)).save(out/'mask.png')
            return {'status':'perception_accepted'}
        with patch('agent.battery_observation.capture',return_value=obs),patch('agent.battery_observation.segment',segment):
            out=observe(self.tools,'d435','observe_head')
        self.assertEqual(out['status'],'localized')
        np.testing.assert_allclose(out['location']['surface_base_mm'],[300,-250,2500],atol=.01)
        self.assertEqual(self.store.state['battery_observations'][obs['observation_id']],out['result_file'])

class WorkflowGateTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.store=RunStore.create(self.tmp.name);self.addCleanup(self.store.close)
        from agent.grasp_tools import WorkflowTools,WORKFLOW_TOOLS
        self.tools=WorkflowTools(Config(),self.store,allowed_tools=WORKFLOW_TOOLS)

    def save_head(self):
        f=frame();f['camera_id']=np.array('d435')
        blob=io.BytesIO();np.savez(blob,**f)
        return save_frame(self.store,blob.getvalue(),source='test',camera='d435')

    def test_rejected_head_cannot_skip_to_wrist_or_pose_and_retry_is_bounded(self):
        obs=self.save_head()
        with patch('agent.battery_observation.capture',return_value=obs),patch(
                'agent.battery_observation.segment',return_value={'status':'needs_new_observation','selection':{'status':'not_found'}}),patch(
                'agent.grasp_pipeline.estimate') as pose:
            first=self.tools.execute('observe_battery',{'camera':'d435'},'head1')
            self.assertEqual(first['data']['status'],'target_not_found')
            for name,args in [('observe_battery',{'camera':'d405'}),
                ('plan_coarse_alignment',{'observation_id':obs['observation_id']}),
                ('preview_battery_grasp',{'observation_id':obs['observation_id'],'clearance_mm':50})]:
                result=self.tools.execute(name,args,name)
                self.assertFalse(result['ok']);self.assertEqual(result['error']['code'],'stage_not_ready')
            self.assertTrue(self.tools.execute('observe_battery',{'camera':'d435'},'head2')['ok'])
            third=self.tools.execute('observe_battery',{'camera':'d435'},'head3')
            self.assertEqual(third['error']['code'],'observation_budget')
            pose.assert_not_called()
        self.assertFalse(self.store.state['pending'])

    def test_failed_recapture_invalidates_old_plan_before_camera_read(self):
        self.store.state.update(grasp_preview='old',coarse_alignment='old',
            latest_battery_observations={'d435':'old','d405':'old'})
        with patch('agent.battery_observation.capture',side_effect=ValueError('camera offline')):
            result=self.tools.execute('observe_battery',{'camera':'d435'},'new')
        self.assertFalse(result['ok'])
        self.assertNotIn('coarse_alignment',self.store.state)
        self.assertNotIn('grasp_preview',self.store.state)
        self.assertFalse(self.store.state['latest_battery_observations'])

    def test_workflow_passes_latest_head_evidence_to_fine_preview(self):
        self.store.state['latest_battery_observations']={'d435':'head','d405':'wrist'}
        self.store.state['battery_observations']={}
        for camera,ident in self.store.state['latest_battery_observations'].items():
            path=self.store.path/(ident+'.json')
            path.write_text(json.dumps(dict(observation_id=ident,camera=camera,status='localized')))
            self.store.state['battery_observations'][ident]=path.name
        result=dict(status='handoff_rejected',message='mismatch',camera_handoff={'identity_confirmed':False})
        with patch('agent.grasp_tools.pipeline',return_value=result) as pipeline:
            out=self.tools.execute('preview_battery_grasp',{'observation_id':'wrist','clearance_mm':50},'fine')
        self.assertTrue(out['ok'])
        self.assertEqual(pipeline.call_args.kwargs['head_observation_id'],'head')
        self.assertEqual(out['data']['camera_handoff'],result['camera_handoff'])
        self.assertFalse(out['data']['executable'])

    def test_depth_failure_preserves_recognition_reason_and_never_localizes(self):
        obs=self.save_head()
        def accepted(store,ident,output,cancelled):
            from PIL import Image
            Image.fromarray(np.zeros((20,30),np.uint8)).save(output/'mask.png')
            return {'status':'perception_accepted'}
        with patch('agent.battery_observation.capture',return_value=obs),patch('agent.battery_observation.segment',accepted):
            result=self.tools.execute('observe_battery',{'camera':'d435'},'depth')
        self.assertEqual(result['data']['status'],'depth_rejected')
        self.assertIsNone(result['data']['location'])
        self.assertIn('深度不足',result['data']['message'])
        self.assertTrue((self.store.path/result['data']['result_file']).is_file())

if __name__=='__main__':unittest.main()
