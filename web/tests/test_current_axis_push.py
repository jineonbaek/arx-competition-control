import unittest
import json,tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
from current_axis_push import CurrentAxisPush,measured_progress,require_arrival,validate_request

class PushTests(unittest.TestCase):
    def test_full_stroke_uses_fewer_stops_with_bounded_joint_interpolation(self):
        from agent.held_insertion_path import plan_insert
        class Model:
            reach=.8
            def wrist_transform(self,q,h):
                T=np.eye(4);T[:3,3]=[.35*np.sin(q[0]),q[1]+.35*(1.-np.cos(q[0])),q[2]];return T
            def inverse_wrist(self,q,h,xyz,rpy,limits):
                end=np.array(q);end[:3]=np.array(xyz)/1000.;end[0]=np.arcsin(end[0]/.35)
                end[1]-=.35*(1.-np.cos(end[0]));return end
            def check(self,a,b,h):return {'clear':True,'lower_mm':20.}
        model=Model();initial=np.zeros(6);limits=[[-180,180]]*6
        timing=dict(tcp_speed_mm_s=8.,tcp_acceleration_mm_s2=4.,joint_speed_deg_s=[15.]*6,joint_acceleration_deg_s2=[18.]*6)
        args=(model,initial,0.,np.zeros(3),np.array([150.,0.,0.]),np.eye(3),np.eye(4),limits)
        opts=dict(fixed_z_axis=[1.,0.,0.],timing_limits=timing)
        old=plan_insert(*args,**opts);new=plan_insert(*args,adaptive_stroke=True,**opts)
        knots=np.array(new['joint_waypoints_rad']);pts=np.array(new['tool_points_mm'])
        np.testing.assert_allclose(pts[[0,-1]],[[0.,0.,0.],[150.,0.,0.]],atol=1e-6)
        self.assertLess(len(knots),len(old['joint_waypoints_rad'])//2)
        self.assertLess(new['timing']['duration_s'],old['timing']['duration_s'])
        for i,(a,b) in enumerate(zip(knots,knots[1:])):
            self.assertLessEqual(np.max(abs(b-a)),np.deg2rad(5.)+1e-10)
            for t in np.linspace(0.,1.,101):
                T=model.wrist_transform(a+t*(b-a),0.)
                self.assertLessEqual(np.linalg.norm(T[:3,3]*1000-(pts[i]+t*(pts[i+1]-pts[i]))),
                                     new['interpolation_position_bound_mm']+1e-8)
        class Blocked(Model):
            def check(self,a,b,h):return {'clear':max(a[0],b[0])<.08,'lower_mm':0.}
        with self.assertRaisesRegex(ValueError,'column'):
            plan_insert(Blocked(),*args[1:],adaptive_stroke=True,**opts)

    def test_adaptive_cad_sweep_refines_interior_even_if_endpoints_are_clear(self):
        from handeye_column import ColumnModel
        model=object.__new__(ColumnModel);model.reach=.8
        model.pose=lambda q,h:((max(0.,.8*abs(q[0]-.04)-.001),'right link'),None)
        start=np.zeros(6);end=start.copy();end[0]=.08
        result=model.check(start,end,0.,adaptive_intervals=True)
        self.assertFalse(result['clear'])
        self.assertGreater(result['steps'],2)
        model.pose=lambda q,h:((.10,'right link'),None)
        result=model.check(start,end,0.,adaptive_intervals=True)
        self.assertTrue(result['clear'])
        self.assertGreater(result['lower_mm'],0.)

    def test_loaded_rail_path_shares_only_identical_poses_within_one_sweep(self):
        from handeye_column import ColumnModel
        model=object.__new__(ColumnModel)
        model.reach=.8
        calls=[]
        def pose(q,height):
            calls.append((q[0],height))
            return ((.10,'right link'),None)
        model.pose=pose
        points=np.zeros((4,6));points[:,0]=[0.,0.,.03,.08]
        self.assertTrue(model.check_rail_path(points,.4))
        self.assertEqual(calls,[(0.,.4),(.03,.4),(.08,.4)])
        model.pose=lambda q,h:((max(0.,.8*abs(q[0]-.055)-.001),'link'),None)
        self.assertFalse(model.check_rail_path(points,.4))
        model.pose=lambda q,h:((0.,'link'),None)
        self.assertFalse(model.check_rail_path(points,.5))

    def test_distance_is_travel_not_depth(self):
        a=np.eye(4);b=a.copy();b[0,3]=.021
        self.assertAlmostEqual(measured_progress(a,b),21.)
        with self.assertRaisesRegex(ValueError,'endpoint'):
            require_arrival(a,b,160.)

    def test_sideways_sag_and_rotation_rejected(self):
        a=np.eye(4);b=a.copy();b[2,3]=.004
        with self.assertRaisesRegex(ValueError,'偏离推进轴'):measured_progress(a,b)
        b=a.copy();b[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
        with self.assertRaisesRegex(ValueError,'末端姿态'):measured_progress(a,b)

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
        for depths,expected in (([-10.,142.,150.],'completed_verified'),([-10.,120.,130.,140.],'completed_unverified'),([23.],'fixed_success'),([23.],'fixed_unknown'),([0.],'fixed_probe')):
            yaw,pitch=np.deg2rad([8.,-4.] if expected=='fixed_probe' else [0.,0.])
            heading=np.array([np.cos(yaw)*np.cos(pitch),np.sin(yaw)*np.cos(pitch),np.sin(pitch)])
            lateral=np.array([-np.sin(yaw),np.cos(yaw),0.])
            rotation=np.column_stack((heading,lateral,np.cross(heading,lateral)))
            class Model:
                reach=.7
                def wrist_transform(self,q,h):
                    from scipy.spatial.transform import Rotation
                    T=np.eye(4);T[:3,:3]=rotation@Rotation.from_euler('x',q[3]).as_matrix()
                    T[:3,3]=heading*q[0]+lateral*q[1];return T
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
                def settled_arm(self,guard):guard();return self.ctrl.snapshot()
                def arm(self,parts,guard,**kwargs):
                    self.ctrl.q=list(parts[-1]['joint_waypoints_rad'][-1])
                    if expected=='fixed_success' and self.ctrl.q[0]>.1:
                        self.ctrl.q[3]=np.radians(42.)
                        guard()
                    if expected in ('fixed_probe','fixed_unknown'):
                        self.ctrl.q[1]=.00267
                        guard()
                    return self.ctrl.snapshot()
                def hold(self):pass
            def plan(*args,**kwargs):
                q=list(args[1]);end=q.copy();delta=args[4]-args[3];end[0]+=float(delta@heading)/1000
                if expected=='fixed_probe':np.testing.assert_allclose(delta,5.*heading,atol=1e-10)
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
                fixed=expected.startswith('fixed_')
                if fixed:
                    def unexpected_observation(*args):
                        self.fail('Full stroke or probe attempted to measure the approach')
                    task.mouth_observer=unexpected_observation
                    def verify_at_end(obs,guard):
                        self.assertNotEqual(expected,'fixed_probe')
                        self.assertAlmostEqual(ctrl.q[0],.150)
                        self.assertNotIn('mouth',obs)
                        confirmed=expected=='fixed_success'
                        return dict(insertion_depth_verified=confirmed,verified_depth_mm=150. if confirmed else None,
                                    handle_outside_mm=45.,verification_frame_ids=['after'],verification_image_time=__import__('time').time(),
                                    verification_reason='视觉复查')
                    task.check_fixed_depth=verify_at_end
                    data=dict(op='start',request_id='fixed',expected_epoch=1,mouth_then_push=True)
                    if expected=='fixed_probe':data['probe_only']=True
                with patch('current_axis_push.plan_insert',side_effect=plan),patch('current_axis_push.DockingMotion',Motion):
                    task.action(data);task.thread.join(4.)
                value=task.snapshot()
                wanted={'fixed_probe':'completed_travel','fixed_success':'completed_verified','fixed_unknown':'held','completed_unverified':'held'}.get(expected,expected)
                self.assertEqual(value['status'],wanted,value.get('reason'))
                self.assertEqual(value['insertion_depth_verified'],wanted=='completed_verified')
                if expected=='fixed_success':self.assertAlmostEqual(value['peak_rotation_drift_deg'],42.)
                if fixed:
                    stages=[(r['phase'],r['distance_mm']) for r in value['stages']]
                    self.assertEqual(stages,[('direct_push',5.)] if expected=='fixed_probe' else [('seat_push',150.)])
                if expected in ('fixed_probe','fixed_unknown'):
                    self.assertAlmostEqual(value['peak_cross_track_mm'],2.67,places=6)
                if expected=='fixed_probe':
                    self.assertFalse(value['released']);self.assertAlmostEqual(ctrl.q[0],.005)
                    self.assertAlmostEqual(value['axis_yaw_deg'],8.);self.assertAlmostEqual(value['axis_pitch_deg'],-4.)
                    self.assertAlmostEqual(value['direct_travel_mm'],5.);self.assertNotIn('mouth_measurement',value)
                    self.assertIsNone(ctrl.token);continue
                if wanted=='held':
                    self.assertFalse(value['released'])
                    self.assertFalse(value.get('returned'))
                    self.assertEqual([c['position'] for c in ctrl.commands],[0])
                    if fixed:self.assertAlmostEqual(ctrl.q[0],.150)
                    else:self.assertGreater(ctrl.q[0],.15)
                    self.assertIsNone(ctrl.token)
                    continue
                self.assertTrue(value['returned'])
                positions=[-3.14] if expected=='completed_verified' else [0,-3.14]
                self.assertEqual([c['position'] for c in ctrl.commands],positions)
                self.assertEqual(value['gripper_commands'],len(positions))
                self.assertEqual(ctrl.commands[0]['type'],'gripper');self.assertAlmostEqual(ctrl.q[0],0.)
                self.assertIsNone(ctrl.token);task.action(data);self.assertEqual(len(ctrl.commands),len(positions))
