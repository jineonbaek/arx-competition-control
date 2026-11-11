import io
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from agent.config import Config
from agent.storage import RunStore, atomic_json
from agent.grasp_tools import GraspTools
from agent.grasp_scene import save_frame
from agent.tests.test_grasp_integration import frame
from agent.coarse_alignment import coarse_geometry, plan

class CoarseTests(unittest.TestCase):
    def test_cad_zero_and_lift_units_not_target_z_directly(self):
        from web.src.handeye_column import ColumnModel, NOMINAL_RAD_PER_M
        model=ColumnModel()
        zero=model.wrist_transform(np.zeros(6),0)[:3,3]*1000
        target=zero+np.array([200,0,300])
        result=coarse_geometry(target,2.,model)
        self.assertAlmostEqual(result['lift']['target_rad'],.2*NOMINAL_RAD_PER_M)
        self.assertAlmostEqual(result['reference_wrist_base_mm'][2],target[2]-100)
        self.assertEqual(result['reference_joints_deg'],[0]*6)
        self.assertIsNone(result['tcp'])
        self.assertFalse(result['executable'])
        self.assertFalse(result['lift']['both_arms_sweep_checked'])

    def test_clamps_both_ends_and_explains_unmet_preference(self):
        from web.src.handeye_column import ColumnModel
        model=ColumnModel()
        for z,limit in ((-1000,0),(3000,18)):
            result=coarse_geometry([500,-250,z],2,model)
            self.assertEqual(result['lift']['target_rad'],limit)
            self.assertTrue(result['lift']['clamped'])
            self.assertGreater(abs(result['lift']['preference_residual_mm']),1)
        for xyz,lift in (([1,2,float('nan')],0),([1,2],0),([1,2,3],19)):
            with self.assertRaises(ValueError):coarse_geometry(xyz,lift,model)

    def test_plan_requires_localized_head_evidence_and_archives_source(self):
        with tempfile.TemporaryDirectory() as directory:
            store=RunStore.create(directory)
            self.addCleanup(store.close)
            tools=GraspTools(Config(),store)
            f=frame();f['camera_id']=np.array('d435')
            blob=io.BytesIO();np.savez(blob,**f)
            obs=save_frame(store,blob.getvalue(),source='fixture',camera='d435')
            ident=obs['observation_id']
            with self.assertRaisesRegex(ValueError,'定位'):plan(tools,ident,'missing')
            path=store.path/'result.json'
            evidence=dict(observation_id=ident,camera='d435',status='localized',
                location=dict(surface_base_mm=[500,-250,750],kind='visible_surface_not_grip'))
            atomic_json(path,evidence)
            store.state['battery_observations']={ident:'result.json'}
            result=plan(tools,ident,'coarse')
            self.assertEqual(result['observation_id'],ident)
            self.assertFalse(result['executable'])
            self.assertTrue((store.path/result['result_file']).is_file())
            evidence['camera']='d405';atomic_json(path,evidence)
            with self.assertRaisesRegex(ValueError,'D435'):plan(tools,ident,'wrong')

if __name__=='__main__':unittest.main()
