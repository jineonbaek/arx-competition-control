import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'web/src'))
from cabinet_execution import CabinetExecutionTask
from competition_modules import PlacementModule
from agent.placement_backend import verify_stability


class Clock:
    def __init__(self):self.t=1.
    def __call__(self):return self.t
    def sleep(self,dt):self.t+=dt


class Model:
    reach=1.
    def wrist_transform(self,q,lift):
        result=np.eye(4);result[:3,3]=q[:3];return result
    def inverse_wrist(self,q,lift,xyz,rpy,limits):return np.r_[np.asarray(xyz)/1000,[0.,0.,0.]]
    def check(self,*args):return dict(clear=True,lower_mm=100.)


class Controller:
    def __init__(self):
        self.clock=Clock();self.owner=None;self.command_epoch=0;self.calls=[];self.q=[0.]*6;self.gripper=-.8
        self.models={'right':Model()};self.config=dict(execution_enabled=True,arms={'right':{
            'gripper_open_raw':-3.,'gripper_limits_raw':[-5.,0.], 'joint_limits_deg':[[-180.,180.]]*6}})
    def snapshot(self):
        arm=dict(fresh=True,joints=self.q[:],gripper=self.gripper,received=self.clock())
        return dict(command_epoch=self.command_epoch,motion_reserved=self.owner is not None,chassis_active=False,
            arms={'right':arm,'left':dict(arm,joints=[0.]*6)},
            body=dict(fresh=True,height=5.,waist=0.,head_yaw=0.,head_pitch=0.,wheels=[0.]*4,received=self.clock()))
    def reserve_motion(self,epoch):
        if self.owner is not None:raise ValueError('busy')
        self.calls.append('reserve');self.owner=object();return self.owner
    def motion_valid(self,token):return token is self.owner and token is not None
    def release_motion(self,token):
        assert self.motion_valid(token);self.calls.append('handoff');self.owner=None
    def start_trajectory(self,side,plan,token,*,preflight):
        assert preflight(self.snapshot(),self.clock())
        self.calls.append('trajectory');self.q=plan['joint_waypoints_rad'][-1]
    def poll_trajectory(self,token,*,preflight):
        assert preflight(self.snapshot(),self.clock());return dict(status='reference_complete')
    def submit(self,data,token,*,preflight=None):
        if preflight:assert preflight(self.snapshot(),self.clock())
        self.calls.append(data['type'])
        if data['type']=='gripper':self.gripper=data['position']
        return dict(ok=True)


class Preview:
    root=ROOT
    def verify_placement(self,*args):return dict(placement_confirmed=True)


def segment(phase,a,b):
    return dict(phase=phase,target_tcp_mm=(np.array(b[:3])*1000).tolist(),target_rotation=np.eye(3).tolist(),
        plan=dict(interpolation='segment_quintic_scalar',time_from_start_s=[0.,1.],joint_waypoints_rad=[a,b]))


class CabinetExecutionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.ctrl=Controller();self.preview=Preview()
        a=[0.]*6;b=[.05,0.,0.,0.,0.,0.];c=[.1,0.,0.,0.,0.,0.]
        self.bundle=dict(tcp_matrix=np.eye(4),observation={'plan':dict(insertion_axis_base=[1.,0.,0.],stroke_mm=50.)},
            timing=dict(joint_speed_deg_s=[3.]*6,joint_acceleration_deg_s2=[6.]*6,
                tcp_speed_mm_s=12.,tcp_acceleration_mm_s2=25.),
            segments=[segment('outside_align',a,b),segment('insert',b,c)])
        self.task=CabinetExecutionTask(self.ctrl,self.preview,directory=self.tmp.name,
            planner=lambda model:copy.deepcopy(self.bundle),auto_worker=False)
        self.task.sleep=self.ctrl.clock.sleep

    def run_task(self):
        self.task.action({'op':'start','model':'demo'});self.task._run();return self.task.snapshot()

    def test_one_start_runs_insert_release_retreat_verify_and_handoff(self):
        calls=[]
        def verify(store,observation,token,check,sleep):
            check();calls.append(self.ctrl.q[0]);return dict(placement_confirmed=True)
        self.task.verifier=verify
        result=self.run_task()
        self.assertEqual(result['status'],'completed',result)
        self.assertTrue(result['battery_placed']);self.assertTrue(result['released'])
        self.assertAlmostEqual(calls[0],.05)
        self.assertEqual(self.ctrl.calls,['reserve','trajectory','trajectory','gripper','trajectory','handoff'])
        self.assertEqual([l['phase'] for l in result['logs']],['outside_align','insert','release','retreat','verify','completed'])

    def test_unknown_release_never_retreats_or_replays(self):
        submit=self.ctrl.submit
        def unknown(data,*args,**kwargs):
            if data['type']=='gripper':raise ValueError('ack lost')
            return submit(data,*args,**kwargs)
        self.ctrl.submit=unknown
        result=self.run_task()
        self.assertEqual(result['status'],'recovery_required')
        self.assertFalse(result['battery_placed']);self.assertFalse(result['released'])
        self.assertEqual(self.ctrl.calls.count('trajectory'),2)
        self.assertIsNone(self.ctrl.owner)
        with self.assertRaises(ValueError):self.task.action({'op':'start','model':'demo'})

    def test_no_open_feedback_means_no_retreat(self):
        original=self.ctrl.submit
        def no_open(data,*args,**kwargs):
            old=self.ctrl.gripper;result=original(data,*args,**kwargs);self.ctrl.gripper=old;return result
        self.ctrl.submit=no_open
        result=self.run_task()
        self.assertEqual(result['status'],'recovery_required');self.assertFalse(result['released'])
        self.assertEqual(self.ctrl.calls.count('trajectory'),2)

    def test_cancel_preserves_grip_stops_and_returns_control(self):
        def sleep(dt):
            self.ctrl.clock.sleep(dt)
            if self.ctrl.clock()>1.04:self.task.action({'op':'cancel'})
        self.task.sleep=sleep
        result=self.run_task()
        self.assertEqual(result['status'],'cancelled');self.assertNotIn('gripper',self.ctrl.calls)
        self.assertIn('arm.stop',self.ctrl.calls);self.assertIsNone(self.ctrl.owner)

    def test_planning_failure_returns_control_without_motion(self):
        def fail(model):raise ValueError('no path')
        self.task.planner=fail
        result=self.run_task()
        self.assertEqual(result['status'],'blocked');self.assertEqual(self.ctrl.calls,['reserve','handoff'])

    def test_visual_failure_does_not_claim_placed(self):
        self.task.verifier=lambda *args:dict(placement_confirmed=False)
        result=self.run_task()
        self.assertEqual(result['status'],'recovery_required');self.assertFalse(result['battery_placed'])

    def test_parent_owns_control_until_total_task_finishes(self):
        token=self.ctrl.reserve_motion(0);module=PlacementModule(self.task)
        context=dict(request_id='run:place',token=token,cancelled=__import__('threading').Event())
        module.start(context);self.task._run()
        self.assertEqual(module.inspect(context)['status'],'completed')
        self.assertIs(self.ctrl.owner,token)
        with self.assertRaises(ValueError):module.start(context)

    def test_browser_cannot_supply_coordinates_or_bypass_checks(self):
        with self.assertRaises(ValueError):self.task.action(dict(op='start',model='demo',target=[0,0,0]))
        self.assertEqual(self.ctrl.calls,[])

    def test_already_at_target_uses_stationary_arrival_without_ik_failure(self):
        self.task.action({'op':'start','model':'demo'})
        self.task.bundle=copy.deepcopy(self.bundle)
        path=self.task._path([0.]*6,np.eye(4))
        self.assertEqual(path['joint_waypoints_rad'],[[0.]*6,[0.]*6])
        self.ctrl.release_motion(self.task.token)

    def test_restart_cannot_replay_an_inserted_but_unreleased_old_task(self):
        import json
        (Path(self.tmp.name)/'receipt.json').write_text(json.dumps(dict(status='completed_hold',model='demo')))
        restored=CabinetExecutionTask(self.ctrl,self.preview,directory=self.tmp.name,auto_worker=False)
        self.assertEqual(restored.snapshot()['status'],'recovery_required')
        self.assertEqual(self.ctrl.calls,[])

    def test_verifier_checks_fresh_frames_for_three_seconds(self):
        clock=Clock()
        result=verify_stability(lambda:dict(image_time=clock(),center_base_m=[0.,0.,0.]),
            lambda:None,clock.sleep,clock)
        self.assertTrue(result['placement_confirmed']);self.assertGreaterEqual(result['stable_duration_s'],3.)
        for observe in [lambda:dict(image_time=1.,center_base_m=[0.,0.,0.]),
                        lambda:dict(image_time=clock(),center_base_m=[clock()*.1,0.,0.])]:
            with self.assertRaises(ValueError):verify_stability(observe,lambda:None,clock.sleep,clock)


if __name__=='__main__':unittest.main()
