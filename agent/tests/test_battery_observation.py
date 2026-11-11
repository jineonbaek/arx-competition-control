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

if __name__=='__main__':unittest.main()
