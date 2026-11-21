import unittest
from unittest.mock import patch

import numpy as np
from observation_sag import at_observation, correct_observation_sag
from agent.held_placement_geometry import loaded_trajectory_plan


class Rig:
    def __init__(self, slope):
        self.slope = slope
        self.models = {'right': self}
        self.config = {'arms': {'right': {'joint_limits_deg': [[-90., 90.]] * 6}}}
        self.adapter = self
        self.taught = np.array([.12, .01, .31, -.4, .02, -1.])
        self.q = self.taught.copy()
        self.stamp = 0
        self.steps = []
        self.records = []

    def wrist_transform(self, q, lift):
        T = np.eye(4)
        T[2, 3] = .5 + lift / 41.54 + self.slope * q[3]
        return T

    def trajectory_status(self, side):
        return dict(ready=True, joint_reference_rad=(self.q + np.arange(6) * .001).tolist())

    def snapshot(self):
        self.stamp += 1
        return dict(arms={'right': dict(joints=self.q.tolist(), received=self.stamp, fresh=True)},
                    body=dict(height=.72, fresh=True))

    def move(self, start, goal):
        ref = self.trajectory_status('right')
        cmd, _ = loaded_trajectory_plan(dict(joint_waypoints_rad=[start, goal]), self.q, ref,
                                        self.config['arms']['right']['joint_limits_deg'])
        np.testing.assert_allclose(np.delete(cmd['joint_waypoints_rad'][-1], 3),
                                   np.delete(ref['joint_reference_rad'], 3), atol=1e-14, rtol=0.)
        self.steps.append((goal-start).copy())
        self.q = goal.copy()

    def run(self, move=None):
        with patch('observation_sag.time.sleep'):
            return correct_observation_sag(self, np.eye(4), self.taught, 1., lambda:None,
                                           move or self.move, self.records.append)


class SagTests(unittest.TestCase):
    def test_repeated_visits_correct_only_j4_for_either_sign(self):
        for slope in (.245, -.245):
            rig = Rig(slope)
            for _ in range(2):
                rig.q[3] -= np.sign(slope) * np.deg2rad(3.)
                self.assertTrue(at_observation(rig, np.eye(4), rig.q, rig.taught, .72, 1.))
                result = rig.run()
                self.assertLess(abs(result['sag_mm']), .01)
                self.assertEqual(len(result['steps']), 2)
                for step in rig.steps:
                    self.assertEqual(np.flatnonzero(step).tolist(), [3])
                    self.assertGreater(slope*step[3], 0.)

    def test_high_and_in_tolerance_never_move(self):
        for shift in (.002, .02, -.003):
            rig=Rig(.245);rig.q[3]+=shift;rig.run()
            self.assertEqual(rig.steps, [])

    def test_stuck_actuator_does_not_accumulate_new_commands(self):
        rig=Rig(.245);rig.q[3]-=.04;calls=[]
        with self.assertRaisesRegex(ValueError, '未改善'):
            rig.run(lambda q,goal:calls.append(goal))
        self.assertEqual(len(calls),1)

class EntryTests(unittest.TestCase):
    def test_stage_carries_corrected_reference_only_after_observation(self):
        import json
        import shutil
        import tempfile
        from pathlib import Path
        from placement_stages import PlacementStages
        from test_placement_stages import Capture, Controller, Model, Push
        class J4Model(Model):
            def wrist_transform(self, q, h):
                T=super().wrist_transform(q,h)
                T[2,3]+=.245*q[3]
                return T
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'config').mkdir()
            (root/'config/right-grip-tcp.json').write_text(json.dumps({'T_wrist_from_tool':np.eye(4).tolist()}))
            shutil.copy(Path(__file__).resolve().parents[2]/'config/battery-execution.json',root/'config')
            c=Controller();c.models['right']=J4Model();c.target=0.
            task=PlacementStages(c,Capture(),Push(),root)
            self.addCleanup(task.close);task.action({'op':'record_preobserve'})
            def follow(motion,segments,*args,**kwargs):
                points=np.array(segments[-1]['joint_waypoints_rad'])
                self.assertEqual(np.flatnonzero(points[-1]-points[0]).tolist(),[3])
                c.q=points[-1].tolist()
                return c.snapshot()
            with patch('placement_stages.DockingMotion.arm',new=follow):
                for request in ('a','b'):
                    c.q[3]=-.025
                    task.action(dict(op='run',phase='preobserve',expected_epoch=c.epoch,request_id=request))
                    task.thread.join(3)
                    s=task.snapshot()
                    self.assertEqual(s['status'],'completed',s.get('reason'))
                    self.assertEqual(len(s['observation_sag']['steps']),1)
                    self.assertLess(abs(s['observation_sag']['sag_mm']),.01)
            self.assertFalse(any(isinstance(command,dict) for command in c.calls))

    def test_combined_route_uses_saved_point_and_completes_j4_before_visual_stage(self):
        from unittest.mock import Mock
        from fast_visual_level import FastVisualLevel, ROOT
        from agent.taught_placement import load_taught_route
        rig=Rig(.245);rig.taught[5]=-np.pi/2;rig.q=rig.taught.copy();rig.q[3]-=.03
        flow=FastVisualLevel.__new__(FastVisualLevel)
        flow.ctrl=rig;flow.tcp=np.eye(4);flow.height=.72;flow.contact=[0,0,0]
        flow.route=load_taught_route(ROOT/'config/taught-placement.json')
        flow.owned_handoff={'outcome':'locked_admitted'}
        flow.saved_observation=lambda:dict(preobserve_joints_deg=np.rad2deg(rig.taught).tolist(),
                                            tolerance_mm=1.,preobserve_source='当前实测点')
        phases=[]
        def timed(name,call,**kw):
            phases.append(name)
            return call()
        flow._timed=timed;flow.guard=lambda:None;flow.save=Mock()
        flow.arm_to=lambda goal,*,start_q:rig.move(start_q,goal)
        flow.auto_d405_horizontal_trim=Mock(side_effect=ValueError('end of observation test'))
        with self.assertRaisesRegex(ValueError,'end of observation test'):
            flow.auto_insert_160_place()
        self.assertEqual(phases,['d405_preobservation','j4_sag_check','d405_horizontal_trim'])
        self.assertEqual(len(rig.steps),1)
