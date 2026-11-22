import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from agent.grasp_path import plan_insert, TimedJointPath
from web.src.handeye_column import ColumnModel


class ThreeSegmentInsertTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ColumnModel()
        cls.cases = json.loads((Path(__file__).parent/'fixtures/grasp-three-segment-poses.json').read_text())

    def plan(self):
        c=self.cases[0];line=c['contact_path']
        return plan_insert(self.model,c['start_joints_rad'],c['lift_rad'],line['pre_mm'],line['end_mm'],
            c['R_base_from_tool'],c['T_wrist_from_tool'],c['limits'],timing_limits=c['timing']['limits'])

    def test_coarse_chord_collision_is_checked_even_when_seed_steps_are_clear(self):
        original=self.model.inverse_wrist_with_clearance
        def clear_seed(*args):
            q,_=original(*args)
            return q,{'clear':True,'lower_mm':100.}
        # Seed solves use their original checker. Only the independently checked
        # coarse command chord fails, so discarded intermediate checks cannot pass it.
        original_check=self.model.check
        def seed_with_real_check(*args):
            with patch.object(self.model,'check',side_effect=original_check):return clear_seed(*args)
        with patch.object(self.model,'inverse_wrist_with_clearance',side_effect=seed_with_real_check), \
             patch.object(self.model,'check',return_value={'clear':False,'lower_mm':0.}):
            with self.assertRaisesRegex(ValueError,'column'):self.plan()

    def test_coarse_joint_curve_bowing_is_rejected(self):
        solutions=[];inverse=self.model.inverse_wrist_with_clearance
        def save(*args):
            result=inverse(*args);solutions.append(result);return result
        with patch.object(self.model,'inverse_wrist_with_clearance',side_effect=save):self.plan()
        original=self.model.wrist_transform
        def bowed(q,lift):
            transform=original(q,lift).copy();transform[2,3]+=.01
            return transform
        # IK still uses the unchanged real inverse solver; corrupted FK evidence
        # between knots must never produce an executable coarse chord.
        with patch.object(self.model,'inverse_wrist_with_clearance',side_effect=solutions), \
             patch.object(self.model,'wrist_transform',side_effect=bowed):
            with self.assertRaisesRegex(ValueError,'2 mm'):self.plan()

    def test_saved_50mm_paths_use_at_most_three_checked_segments(self):
        for case in self.cases:
            with self.subTest(pose=case['label']):
                line=case['contact_path']; tcp=np.asarray(case['T_wrist_from_tool'])
                result=plan_insert(self.model,case['start_joints_rad'],case['lift_rad'],
                    line['pre_mm'],line['end_mm'],case['R_base_from_tool'],tcp,case['limits'],
                    timing_limits=case['timing']['limits'])
                points=np.asarray(result['joint_waypoints_rad'])
                self.assertLessEqual(len(points)-1,3)
                connector=not np.array_equal(points[0],case['start_joints_rad'])
                self.assertLessEqual(len(points)-1+int(connector),3)
                self.assertLess(result['timing']['duration_s'],case['timing']['duration_s'])
                self.assertTrue(result['column_checked'])
                for a,b in zip(points,points[1:]):
                    self.assertTrue(self.model.check(a,b,case['lift_rad'])['clear'])
                path=TimedJointPath(points,result['timing']['reach_bound_m'],
                    tcp_position=lambda q:(self.model.wrist_transform(q,case['lift_rad'])@tcp)[:3,3],
                    **case['timing']['limits'])
                start=np.asarray(line['pre_mm']);axis=np.asarray(line['direction_base'])
                previous=-1.
                for t in np.linspace(0,path.times[-1],401):
                    q,v,a=path.sample(t); actual=self.model.wrist_transform(q,case['lift_rad'])@tcp
                    delta=actual[:3,3]*1000-start;along=float(delta@axis)
                    self.assertLessEqual(np.linalg.norm(delta-along*axis),2.)
                    self.assertGreaterEqual(along,previous-1e-3);previous=along
                    self.assertLessEqual(np.degrees(Rotation.from_matrix(np.asarray(case['R_base_from_tool']).T@actual[:3,:3]).magnitude()),.7)
                    self.assertTrue((np.abs(v)<=np.radians(case['timing']['limits']['joint_speed_deg_s'])+1e-9).all())
                    self.assertTrue((np.abs(a)<=np.radians(case['timing']['limits']['joint_acceleration_deg_s2'])+1e-9).all())
                self.assertAlmostEqual(previous,50.,delta=.5)
                times=np.linspace(0,path.times[-1],2001)
                xyz=np.array([(self.model.wrist_transform(path.sample(t)[0],case['lift_rad'])@tcp)[:3,3]*1000 for t in times])
                dt=times[1]-times[0]
                self.assertLessEqual(np.linalg.norm(np.diff(xyz,axis=0)/dt,axis=1).max(),case['timing']['limits']['tcp_speed_mm_s'])
                self.assertLessEqual(np.linalg.norm(np.diff(xyz,n=2,axis=0)/dt**2,axis=1).max(),case['timing']['limits']['tcp_acceleration_mm_s2'])


if __name__=='__main__':unittest.main()
