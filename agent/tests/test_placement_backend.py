import unittest
import numpy as np
from agent import placement_backend as b

class ReleaseGeometryTests(unittest.TestCase):
    def region(self):return dict(target_base_m=[0,0,0],normal_base=[0,0,1],forward_axis_base=[1,0,0],left_axis_base=[0,1,0],xy_tolerance_mm=30,z_tolerance_mm=3,yaw_tolerance_deg=8)
    def test_horizontal_tolerance_never_allows_drop_from_height(self):
        self.assertTrue(hasattr(b,'release_geometry'),'support-height release gate is required')
        t=np.eye(4);t[:3,3]=[.020,.020,.020]
        r=b.release_geometry(t,self.region(),[[-.04,-.04,0],[.04,.04,.195]])
        self.assertFalse(r['ready']);self.assertGreater(r['height_error_mm'],3)
    def test_upright_bottom_on_plane_is_ready(self):
        r=b.release_geometry(np.eye(4),self.region(),[[-.04,-.04,0],[.04,.04,.195]])
        self.assertTrue(r['ready'])
    def test_tilted_battery_corner_below_plane_rejected(self):
        t=np.eye(4);a=.12;t[:3,:3]=[[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]]
        self.assertFalse(b.release_geometry(t,self.region(),[[-.04,-.04,0],[.04,.04,.195]])['ready'])
    def test_nonfinite_and_bad_transform_rejected(self):
        for v in [np.full((4,4),np.nan),np.zeros((4,4))]:
            with self.assertRaises(ValueError):b.release_geometry(v,self.region(),[[-.04,-.04,0],[.04,.04,.195]])

if __name__=='__main__':unittest.main()

class Clock:
    def __init__(self):self.t=0.
    def __call__(self):return self.t
    def sleep(self,dt):self.t+=dt

class Model:
    reach=1.
    def wrist_transform(self,q,lift):
        from scipy.spatial.transform import Rotation
        t=np.eye(4);t[:3,3]=q[:3];t[:3,:3]=Rotation.from_rotvec(q[3:]).as_matrix();return t
    def inverse_wrist(self,q,lift,xyz,rpy,limits):
        from scipy.spatial.transform import Rotation
        return np.r_[np.asarray(xyz)/1000,Rotation.from_euler('xyz',rpy,degrees=True).as_rotvec()]
    def check(self,a,b,lift):return dict(clear=True,lower_mm=100.)

class Controller:
    def __init__(self):
        self.models={'right':Model()};self.q=np.array([.4,0,.3,0,0,0]);self.gripper=-.4;self.stamp=0;self.interrupted=False;self.submissions=[];self.trajectories=[]
        self.config={'arms':{'right':{'joint_limits_deg':[[-180,180]]*6,'gripper_open_raw':-3.14}}}
    def motion_valid(self,token):return not self.interrupted
    def snapshot(self):
        self.stamp+=.1
        return dict(arms={'right':dict(fresh=True,joints=self.q.tolist(),gripper=self.gripper,timestamp=self.stamp,received=self.clock() if hasattr(self,'clock') else 0.),
            'left':dict(fresh=True,joints=[0]*6)},body=dict(fresh=True,height=1.,received=self.clock() if hasattr(self,'clock') else 0.),chassis_active=False,motion_interrupted=self.interrupted)
    def start_trajectory(self,side,plan,token,preflight):
        if set(plan)!={'interpolation','time_from_start_s','joint_waypoints_rad'}:raise ValueError('existing driver wire schema mismatch')
        if not preflight(self.snapshot(),0):raise ValueError('preflight')
        self.trajectories.append(plan);self.q=np.array(plan['joint_waypoints_rad'][-1])
    def poll_trajectory(self,token,preflight):
        if not preflight(self.snapshot(),0):raise ValueError('preflight')
        return dict(status='reference_complete')
    def submit(self,data,token,preflight=None):
        if preflight and not preflight(self.snapshot(),0):raise ValueError('release denied')
        self.submissions.append(data)
        if data['type']=='gripper':self.gripper=data['position']
        return dict(ok=True)
    def interrupt_motion(self,token):self.interrupted=True

class BackendTransportTests(unittest.TestCase):
    def make(self):
        from types import SimpleNamespace
        from pathlib import Path
        import tempfile
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        store=SimpleNamespace(cancelled=False,path=Path(tmp.name),state={'placement_flow':{'phase':'descend'}})
        c=Controller();p=dict(pick={'hold_gripper_range':[-.7,-.1],'gripper_policy':{'tolerance':[.1],'max_speed':[.1],'settle_s':.5,'timeout_s':10.,'stall_s':3.,'max_age_s':1.},'arm_policy':{'tolerance':[18]*3+[5]*3,'max_speed':[2]*3+[1]*3,'settle_s':.5,'timeout_s':10.,'stall_s':3.,'max_age_s':1.},'max_pair_skew_s':.2,'lift_tolerance_rad':.02},
            timing_limits={'joint_speed_deg_s':[5]*6,'joint_acceleration_deg_s2':[10]*6,'tcp_speed_mm_s':20,'tcp_acceleration_mm_s2':30})
        backend=b.PlacementBackend(store,c,object(),{},p,None,environment_clear=True)
        backend.held=dict(bounds_m=[[-.04,-.04,0],[.04,.04,.195]],T_wrist_from_battery=np.eye(4).tolist())
        backend.region=ReleaseGeometryTests().region();backend.region.update(image_time=__import__('time').time(),camera='d435',center_base_m=[.4,0,.2],radius_m=.2); backend.region['target_base_m']=[.4,0,.2];backend.head=backend.region
        backend.lift=1.;backend.left_joints=np.zeros(6);clock=Clock();backend.clock=clock;backend.sleep=clock.sleep;c.clock=clock
        import copy,time
        backend.preview=SimpleNamespace(capture=lambda *a,**kw:{**copy.deepcopy(backend.region),'image_time':time.time()},check_payload_support=lambda *a:None)
        return backend,c,clock
    def test_timed_descent_uses_existing_exact_driver_schema(self):
        backend,c,clock=self.make();backend._move(0.)
        self.assertEqual(len(c.trajectories),1);self.assertGreater(clock.t,.49)
        self.assertAlmostEqual(c.q[2],.2);self.assertEqual(c.submissions,[])
    def test_already_at_preplace_still_settles_without_resending_motion(self):
        backend,c,clock=self.make();c.q[2]=.28;backend._move(.08)
        self.assertEqual(c.trajectories,[]);self.assertGreaterEqual(clock.t,.5)
    def test_release_rejected_above_support_even_within_xy_tolerance(self):
        backend,c,_=self.make()
        with self.assertRaises(ValueError):backend._release()
        self.assertEqual(c.submissions,[])
    def test_release_requires_measured_open_feedback(self):
        backend,c,clock=self.make();c.q[2]=.2;backend._release()
        self.assertTrue(backend.released);self.assertEqual(c.submissions[0]['type'],'gripper');self.assertGreaterEqual(clock.t,.5)
    def test_cancelled_task_cannot_send_any_command(self):
        backend,c,_=self.make();backend.store.cancelled=True
        with self.assertRaises(ValueError):backend._move(0.)
        self.assertEqual(c.trajectories,[])
    def test_lift_or_holding_change_aborts(self):
        backend,c,_=self.make();backend.lift=2.
        with self.assertRaises(ValueError):backend._check()
        backend.lift=1.;c.gripper=-3.14
        with self.assertRaises(ValueError):backend._check()
    def test_overlong_trajectory_is_rejected_before_publish(self):
        backend,c,_=self.make();backend.pick_policy['timing_limits']['joint_speed_deg_s']=[.001]*6
        with self.assertRaises(ValueError):backend._move(0.)
        self.assertEqual(c.trajectories,[])
    def test_oscillation_inside_wide_tolerance_never_settles(self):
        backend,c,clock=self.make();c.q[2]=.2;original=c.snapshot
        def oscillating():
            c.q[1]=.020 if int(clock.t/.051)%2 else -.020
            return original()
        c.snapshot=oscillating
        with self.assertRaises(ValueError):backend._move(0.)
        self.assertFalse(any(x['type']=='gripper' for x in c.submissions))
    def test_release_rechecks_a_new_support_observation(self):
        backend,c,clock=self.make();c.q[2]=.2
        from unittest.mock import Mock
        backend._refresh_support=Mock(side_effect=ValueError('old support'))
        with self.assertRaises(ValueError):backend._release()
        self.assertEqual(c.submissions,[])
    def test_expired_region_cannot_publish(self):
        backend,c,_=self.make();backend.region['image_time']=0
        with self.assertRaises(ValueError):backend._move(0.)
        self.assertEqual(c.trajectories,[])
    def test_withdrawal_after_opening_does_not_require_closed_gripper(self):
        backend,c,_=self.make();backend.released=True;c.gripper=-3.14;c.q[2]=.2
        backend._move(.08,holding=False)
        self.assertEqual(len(c.trajectories),1)
    def test_stale_received_time_cannot_be_counted_as_stable(self):
        backend,c,clock=self.make();c.q[2]=.2;original=c.snapshot
        def stale():
            s=original();s['arms']['right']['received']=0.;s['body']['received']=0.;return s
        c.snapshot=stale
        with self.assertRaises(ValueError):backend._move(0.)
    def test_monotonic_drift_inside_wide_tolerance_is_not_rest(self):
        backend,c,clock=self.make();c.q[2]=.2;original=c.snapshot
        def drift():c.q[1]=.020*clock.t;return original()
        c.snapshot=drift
        with self.assertRaises(ValueError):backend._move(0.)
    def test_release_preflight_rejects_motion_after_the_rest_window(self):
        backend,c,clock=self.make();c.q[2]=.2;original=c.submit
        def moved(data,token,preflight=None):c.q[1]=.02;return original(data,token,preflight)
        c.submit=moved
        with self.assertRaises(ValueError):backend._release()
        self.assertEqual(c.submissions,[])

class HeldEvidenceTests(unittest.TestCase):
    def fixture(self):
        import tempfile,json,hashlib
        from pathlib import Path
        t=tempfile.TemporaryDirectory();self.addCleanup(t.cleanup);site=Path(t.name)
        root=site/'agent/runs';run=root/'run-1';run.mkdir(parents=True)
        digest=lambda raw:hashlib.sha256(raw).hexdigest()
        dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('extra','<u2')]);tri=np.zeros(4,dtype=dtype)
        tri['vertices'][:]=[[[-.04,-.04,0],[.04,.04,.195],[0,0,.1]]]*4
        mesh=site/'battery.stl';mesh.write_bytes(bytes(80)+(4).to_bytes(4,'little')+tri.tobytes());profile={'mesh_sha256':{'battery-print.stl':digest(mesh.read_bytes())}}
        files={'web/config/control.json':b'{}','config/right-grip-tcp.json':b'{}','web/public/models/lift2/diy-20261105/urdf/lift2_with_sensors.urdf':b'<robot/>','web/public/cad/battery-grip.json':json.dumps(profile).encode()}
        for name,data in files.items():p=site/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
        plan={'joint_waypoints_rad':[[0]*6,[0]*6],'lift_rad':1.,'calibration_id':'tcp:'+digest(b'{}'),
            'control_config_sha256':digest(files['web/config/control.json']),'urdf_sha256':digest(files['web/public/models/lift2/diy-20261105/urdf/lift2_with_sensors.urdf']),
            'profile_sha256':digest(files['web/public/cad/battery-grip.json'])}
        preview={'status':'preview_complete','quality':{'status':'consistent'},'pose':{'mesh_sha256':digest(mesh.read_bytes()),'T_base_from_object':np.eye(4).tolist()}}
        state={'battery_flow':{'status':'completed','grasp_confirmed':True},'pending':None,'grasp_preview':'preview.json','battery_pick_plan':'plan.json'}
        for key,value in [('preview',preview),('plan',plan)]:
            p=run/(key+'.json');p.write_text(json.dumps(value));state['grasp_preview_sha256' if key=='preview' else 'battery_pick_plan_sha256']=digest(p.read_bytes())
        (run/'state.json').write_text(json.dumps(state));return root,mesh,profile,site
    def test_completed_grasp_and_current_versions_are_bound(self):
        root,mesh,profile,_=self.fixture();r=b.load_held(root,Model(),mesh,profile);self.assertEqual(r['run_id'],'run-1')
    def test_changed_tcp_or_urdf_is_rejected(self):
        for name in ['config/right-grip-tcp.json','web/public/models/lift2/diy-20261105/urdf/lift2_with_sensors.urdf']:
            root,mesh,profile,site=self.fixture();(site/name).write_text('changed')
            with self.assertRaisesRegex(ValueError,'标定|配置|模型|版本'):b.load_held(root,Model(),mesh,profile)
    def test_newer_failed_grasp_is_not_skipped(self):
        import json,time,os
        root,mesh,profile,site=self.fixture();p=root/'run-2';p.mkdir();(p/'state.json').write_text(json.dumps({'battery_flow':{'status':'needs_attention'}}));os.utime(p/'state.json',(time.time()+.01,time.time()+.01))
        with self.assertRaises(ValueError):b.load_held(root,Model(),mesh,profile)

class PostPlacementVisionTests(unittest.TestCase):
    make=BackendTransportTests.make
    def test_success_requires_four_new_frames_spanning_three_seconds(self):
        backend,c,clock=self.make();backend.released=True;c.gripper=-3.14
        import copy
        backend.wall_clock=lambda:1000.+clock.t
        backend.region['image_time']=1000.;backend.head=copy.deepcopy(backend.region)
        kinds=[]
        def capture(*args,**kwargs):
            clock.sleep(.1);return {**copy.deepcopy(backend.region),'image_time':backend.wall_clock()}
        def verify(*args,**kwargs):kinds.append(kwargs.get('recognize',True));return {'center_base_m':[.4,0,.3]}
        backend.preview.capture=capture;backend.preview.verify_battery=verify
        result=backend.call('verify');self.assertEqual(result['status'],'confirmed');self.assertEqual(kinds.count(True),1);self.assertGreaterEqual(kinds.count(False),4)
