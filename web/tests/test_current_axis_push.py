import unittest
import json,tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
from current_axis_push import CurrentAxisPush,measured_progress,require_arrival,validate_request

class PushTests(unittest.TestCase):
    def test_distance_is_travel_not_depth(self):
        a=np.eye(4);b=a.copy();b[0,3]=.021
        self.assertAlmostEqual(measured_progress(a,b),21.)
        with self.assertRaisesRegex(ValueError,'endpoint'):
            require_arrival(a,b,160.)

    def test_sideways_sag_and_rotation_rejected(self):
        a=np.eye(4);b=a.copy();b[2,3]=.004
        with self.assertRaisesRegex(ValueError,'corridor'):measured_progress(a,b)
        b=a.copy();b[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
        with self.assertRaisesRegex(ValueError,'rotation'):measured_progress(a,b)

    def test_visual_axis_preserves_world_height_on_tilted_base(self):
        axis=np.array([np.cos(.04),0.,-np.sin(.04)])
        a=np.eye(4);b=a.copy();b[:3,3]=axis*.020
        self.assertAlmostEqual(measured_progress(a,b,axis),20.)
        require_arrival(a,b,20.,axis)

    def test_exact_start_payload_and_replay_identity(self):
        validate_request({'op':'start','request_id':'one','expected_epoch':13})
        validate_request({'op':'start','request_id':'one','expected_epoch':13,'distance_mm':120.,'speed_mm_s':10.,
                          'axis_base':[1.,0.,0.],'height_up_base':[0.,0.,1.]})
        for data in ({'op':'start','request_id':'one','expected_epoch':True},
                     {'op':'start','request_id':'one','expected_epoch':13,'release':True},
                     {'op':'start','request_id':'one','expected_epoch':13,'distance_mm':999.},
                     {'op':'start','request_id':'one','expected_epoch':13,'axis_base':[1.,0.,0.]}):
            with self.assertRaises(ValueError):validate_request(data)

    def test_unreachable_tail_does_not_dispatch_or_leak_ownership(self):
        root=Path(__file__).resolve().parents[2]
        class Model:
            def wrist_transform(self,q,h):return np.eye(4)
        class Controller:
            config=json.loads((root/'web/config/control.json').read_text())
            models={'right':Model()}
            token=None
            calls=[]
            def __init__(self):self.adapter=self
            def trajectory_status(self,side):return {'driver_instance':'test','command_epoch':1,'joint_reference_rad':[0.]*6}
            def snapshot(self):return dict(command_epoch=1,motion_reserved=self.token is not None,
                competition_reserved=False,chassis_active=False,body={'height':.4,'fresh':True},
                arms={k:dict(joints=[0.]*6,gripper=-6.9,fresh=True) for k in ('left','right')})
            def reserve_motion(self,epoch):self.token=object();self.calls.append('reserve');return self.token
            def motion_valid(self,token):return self.token is token
            def release_motion(self,token):self.calls.append('release');self.token=None
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);(folder/'config').mkdir()
            for name in ('right-grip-tcp.json','battery-execution.json'):
                (folder/'config'/name).write_bytes((root/'config'/name).read_bytes())
            ctrl=Controller();task=CurrentAxisPush(ctrl,folder)
            data={'op':'start','request_id':'one','expected_epoch':1}
            with patch('current_axis_push.plan_insert',side_effect=ValueError('unreachable tail')):
                task.action(data);task.thread.join(2.)
            self.assertEqual(task.snapshot()['status'],'blocked')
            self.assertFalse(task.snapshot()['control_retained'])
            self.assertEqual(ctrl.calls,['reserve','release'])
            task.action(data)
            self.assertEqual(ctrl.calls,['reserve','release'])

    def test_native_completion_requires_requested_measured_distance(self):
        root=Path(__file__).resolve().parents[2]
        for actual_distance,expected_status in ((.120,'completed_travel'),(.110,'held')):
            pose=[0.]
            class Model:
                def wrist_transform(self,q,h):
                    T=np.eye(4);T[0,3]=pose[0];return T
            class Ctrl:
                config=json.loads((root/'web/config/control.json').read_text());models={'right':Model()}
                def __init__(self):self.adapter=self;self.token=None
                def trajectory_status(self,side):return dict(driver_instance='test',command_epoch=1,joint_reference_rad=[0.]*6,status='idle')
                def snapshot(self):return dict(command_epoch=1,motion_reserved=self.token is not None,competition_reserved=False,chassis_active=False,
                    arms={s:dict(joints=[0.]*6,gripper=-.5,fresh=True,received=__import__('time').monotonic()) for s in ('left','right')},body=dict(height=.4,fresh=True))
                def reserve_motion(self,epoch):self.token=object();return self.token
                def motion_valid(self,token):return token is self.token
                def release_motion(self,token):self.token=None
            class Motion:
                handoffs=[]
                def __init__(self,ctrl,*args,**kwargs):self.ctrl=ctrl
                def arm(self,*args,**kwargs):pose[0]=actual_distance;return self.ctrl.snapshot()
                def hold(self):pass
            with tempfile.TemporaryDirectory() as tmp:
                folder=Path(tmp);(folder/'config').mkdir()
                for name in ('right-grip-tcp.json','battery-execution.json'):(folder/'config'/name).write_bytes((root/'config'/name).read_bytes())
                ctrl=Ctrl();task=CurrentAxisPush(ctrl,folder)
                plan=dict(joint_waypoints_rad=[[0.]*6,[.01]*6],timing={'time_from_start_s':[0.,2.]})
                with patch('current_axis_push.plan_insert',return_value=plan),patch('current_axis_push.DockingMotion',Motion):
                    task.action(dict(op='start',request_id='distance',expected_epoch=1,distance_mm=120.));task.thread.join(3.)
                self.assertEqual(task.snapshot()['status'],expected_status)
                self.assertFalse(task.snapshot()['insertion_depth_verified']);self.assertIsNone(ctrl.token)

if __name__=='__main__':unittest.main()

class VerifiedPushTests(unittest.TestCase):
    def test_verified_and_unverified_attempts_release_exit_once_without_replaying(self):
        root=Path(__file__).resolve().parents[2]
        for depths,expected in (([-10.,142.,150.],'completed_verified'),([-10.,120.,130.,140.],'completed_unverified')):
            class Model:
                reach=.7
                def wrist_transform(self,q,h):
                    T=np.eye(4);T[0,3]=q[0];return T
            class Ctrl:
                config=json.loads((root/'web/config/control.json').read_text());models={'right':Model()}
                def __init__(self):self.adapter=self;self.token=None;self.q=[0.]*6;self.grip=-.5;self.target=0 if expected=='completed_verified' else -1.247;self.commands=[]
                def trajectory_status(self,side):return dict(driver_instance='test',command_epoch=1,ready=True,enabled=True,gripper_target_raw=self.target,joint_reference_rad=self.q,status='idle')
                def snapshot(self):return dict(command_epoch=1,motion_reserved=self.token is not None,competition_reserved=False,chassis_active=False,
                    arms={s:dict(joints=self.q if s=='right' else [0.]*6,gripper=self.grip,fresh=True,received=__import__('time').monotonic()) for s in ('left','right')},body=dict(height=.4,fresh=True))
                def reserve_motion(self,epoch):self.token=object();return self.token
                def motion_valid(self,token):return token is self.token
                def release_motion(self,token):self.token=None
                def submit(self,command,token,**kwargs):
                    if 'preflight' in kwargs:kwargs['preflight'](self.snapshot(),None)
                    self.commands.append(command);self.target=command['position']
                    if self.target!=0:self.grip=self.target
                    return {'ok':True}
            class Motion:
                handoffs=[]
                def __init__(self,ctrl,*args,**kwargs):self.ctrl=ctrl
                def arm(self,parts,*args,**kwargs):self.ctrl.q=list(parts[-1]['joint_waypoints_rad'][-1]);return self.ctrl.snapshot()
                def hold(self):pass
            def plan(*args,**kwargs):
                q=list(args[1]);end=q.copy();end[0]+=(args[4][0]-args[3][0])/1000
                return dict(joint_waypoints_rad=[q,end],timing={'time_from_start_s':[0.,1.]})
            values=iter(depths);index=[0]
            def observe(reference):
                index[0]+=1
                return dict(frame_id=str(index[0]),image_time=__import__('time').time(),calibration_sha256='a'*64,
                            depth_mm=next(values),axis_base=[1.,0.,0.],up_base=[0.,0.,1.])
            with tempfile.TemporaryDirectory() as tmp:
                folder=Path(tmp);(folder/'config').mkdir()
                for name in ('right-grip-tcp.json','battery-execution.json'):(folder/'config'/name).write_bytes((root/'config'/name).read_bytes())
                ctrl=Ctrl();task=CurrentAxisPush(ctrl,folder,observer=observe)
                data=dict(op='start',request_id='verified',expected_epoch=1,verify_insertion=True,cabinet_depth_mm=150.)
                with patch('current_axis_push.plan_insert',side_effect=plan),patch('current_axis_push.DockingMotion',Motion):
                    task.action(data);task.thread.join(4.)
                value=task.snapshot();self.assertEqual(value['status'],expected,value.get('reason'))
                self.assertEqual(value['insertion_depth_verified'],expected=='completed_verified')
                self.assertTrue(value['returned'])
                positions=[-3.14] if expected=='completed_verified' else [0,-3.14]
                self.assertEqual([c['position'] for c in ctrl.commands],positions)
                self.assertEqual(value['gripper_commands'],len(positions))
                self.assertEqual(ctrl.commands[0]['type'],'gripper');self.assertAlmostEqual(ctrl.q[0],0.)
                self.assertIsNone(ctrl.token);task.action(data);self.assertEqual(len(ctrl.commands),len(positions))
