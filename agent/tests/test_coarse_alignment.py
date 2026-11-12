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
from agent.coarse_alignment import coarse_geometry, observer_search_geometry, plan

class CoarseTests(unittest.TestCase):
    def test_fixed_lift_ignores_target_z_and_defers_xy_to_d405(self):
        from web.src.handeye_column import ColumnModel, NOMINAL_RAD_PER_M
        model=ColumnModel()
        zero=model.wrist_transform(np.zeros(6),0)[:3,3]*1000
        target=zero+np.array([200,0,300])
        limits=[[-90,90],[-6,206],[-6,172],[-74,74],[-85,85],[-100,100]]
        result=coarse_geometry(target,5.02,model,fixed_lift_rad=5.02,
            observer_start_z_mm=700.5,observer_rpy_deg=[-.3,-2.6,.1],
            current_joints=np.zeros(6),limits_deg=limits)
        self.assertEqual(result['lift']['target_rad'],5.02)
        self.assertAlmostEqual(result['lift']['delta_rad'],0)
        self.assertTrue(result['lift']['fixed'])
        self.assertAlmostEqual(result['reference_wrist_base_mm'][2],
            zero[2]+5.02/NOMINAL_RAD_PER_M*1000)
        self.assertEqual(result['x_y_alignment'],'deferred_to_d405')
        self.assertEqual(result['reference_joints_deg'],[0]*6)
        self.assertIsNone(result['tcp'])
        self.assertFalse(result['executable'])
        self.assertFalse(result['lift']['both_arms_sweep_checked'])
        self.assertEqual(result['observer']['status'],'fixed_observer_candidate')
        self.assertAlmostEqual(result['observer']['target_wrist_base_mm'][2],700.5)
        self.assertTrue(result['observer']['column_path']['clear'])

    def test_rejects_invalid_fixed_lift_and_inputs(self):
        from web.src.handeye_column import ColumnModel
        model=ColumnModel()
        limits=[[-90,90],[-6,206],[-6,172],[-74,74],[-85,85],[-100,100]]
        low=coarse_geometry([500,-250,-1000],0,model,fixed_lift_rad=0,
            observer_start_z_mm=700.5,observer_rpy_deg=[0,0,0],current_joints=np.zeros(6),limits_deg=limits)
        high=coarse_geometry([500,-250,3000],18,model,fixed_lift_rad=18,
            observer_start_z_mm=700.5,observer_rpy_deg=[0,0,0],current_joints=np.zeros(6),limits_deg=limits)
        self.assertEqual((low['lift']['target_rad'],high['lift']['target_rad']),(0,18))
        for xyz,lift,fixed in (([1,2,float('nan')],0,5.02),([1,2],0,5.02),
                               ([1,2,3],19,5.52),([1,2,3],0,-1),([1,2,3],0,19)):
            with self.assertRaises(ValueError):coarse_geometry(xyz,lift,model,fixed_lift_rad=fixed,
                observer_start_z_mm=700.5,observer_rpy_deg=[0,0,0],current_joints=np.zeros(6),limits_deg=limits)

    def test_plan_requires_localized_head_evidence_and_uses_fixed_lift(self):
        with tempfile.TemporaryDirectory() as directory:
            store=RunStore.create(directory)
            self.addCleanup(store.close)
            tools=GraspTools(Config(),store)
            f=frame();f['camera_id']=np.array('d435');f['joints']=np.zeros(6);f['left_joints']=np.zeros(6)
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
            self.assertEqual(result['lift']['column_sweep']['status'],'unavailable')
            self.assertFalse(result['lift']['column_sweep']['both_arms_column_checked'])
            self.assertEqual(result['lift']['target_rad'],5.02)
            self.assertTrue(result['lift']['fixed'])
            self.assertEqual(result['observer']['status'],'fixed_observer_candidate')
            self.assertEqual(len(result['observer']['joints_rad']),6)
            self.assertTrue((store.path/result['result_file']).is_file())
            evidence['camera']='d405';atomic_json(path,evidence)
            with self.assertRaisesRegex(ValueError,'D435'):plan(tools,ident,'wrong')

    def test_fixed_lift_plan_does_not_require_pre_lift_observer_or_tcp(self):
        with tempfile.TemporaryDirectory() as directory:
            store=RunStore.create(directory);self.addCleanup(store.close)
            f=frame();f.update(camera_id=np.array('d435'),joints=np.zeros(6))
            blob=io.BytesIO();np.savez(blob,**f)
            obs=save_frame(store,blob.getvalue(),source='fixture',camera='d435')
            ident=obs['observation_id']
            evidence=dict(observation_id=ident,camera='d435',status='localized',
                location=dict(surface_base_mm=[500,-250,750],kind='visible_surface_not_grip',
                    surface_bounds_p90_base_mm=[[460,-290,690],[540,-210,810]]))
            atomic_json(store.path/'result.json',evidence)
            store.state['battery_observations']={ident:'result.json'}
            result=plan(GraspTools(Config(),store),ident,'coarse')
            self.assertEqual(result['lift']['target_rad'],5.02)
            self.assertEqual(result['observer']['status'],'fixed_observer_candidate')
            self.assertTrue(result['observer']['column_path']['clear'])
            self.assertFalse(result['executable']);self.assertIsNone(result['tcp'])
            self.assertFalse(result['ik_checked'])

    def test_observer_search_uses_absolute_z_offset_and_checked_joint_path(self):
        from web.src.handeye_column import ColumnModel
        model=ColumnModel();limits=[[-90,90],[-6,206],[-6,172],[-74,74],[-85,85],[-100,100]]
        q=np.zeros(6);observer=coarse_geometry([250,-249,442],5.02,model,fixed_lift_rad=5.02,
            observer_start_z_mm=700.5,observer_rpy_deg=[-.3,-2.6,.1],current_joints=q,
            limits_deg=limits)['observer']
        result=observer_search_geometry(observer,10.,q,5.02,model,limits)
        self.assertAlmostEqual(result['target_wrist_base_mm'][2],710.5)
        self.assertTrue(result['clear'])
        self.assertEqual(np.asarray(result['joints_rad']).shape,(6,))

if __name__=='__main__':unittest.main()
