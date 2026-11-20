import tempfile
import unittest
from pathlib import Path
from ros_adapter import RosAdapter
from visual_docking_live import LiveDockTask,LiveDockBackend,split_plan,validate_arrival,scene_drift_allowance,level_first_path


def status():
    return dict(driver_instance='driver',command_epoch=0,enabled=True,request_id='',status='idle',reason='',ready=True,manual_joint_active=False,
                joint_reference_rad=[0.]*6,gripper_target_raw=0.,reference_from_trajectory=False)


class LiveTests(unittest.TestCase):
    def test_changed_final_scene_is_rechecked_geometrically_not_blanket_margin_bypass(self):
        import numpy as np,time
        from types import SimpleNamespace
        from agent.tests.test_cabinet_loaded_sweep import Model
        b=LiveDockBackend.__new__(LiveDockBackend)
        b.ctrl=SimpleNamespace(models=dict(right=Model(-1.),left=Model(1.)))
        b.tcp=np.eye(4);b.tcp[0,3]=.3
        b.loaded_battery=dict(bounds_mm=[[-40.,-37.5,0.],[40.,37.5,195.]],contact_mm=[0.,0.,169.])
        b.cached=dict(strategy='arm',points=[[0.]*6,[0.]*6])
        state=dict(arms=dict(left=dict(joints=[0.]*6)),body=dict(height=0.))
        scene=dict(window=dict(center_base_m=[1.,-1.,0.],outward_normal_base=[-1.,0.,0.]),
                   table_estimate=dict(status='inferred',front_normal_base=[1.,0.,0.],front_offset_m=-.1,top_m=-.05,bottom_m=-.55))
        b.recheck_final_scene(state,scene,dict(image_time=time.time()),at_entry=False,corner_change=2.,remaining=.5)
        self.assertAlmostEqual(b.cached['sweep']['lower_mm']['table'],12.5)
        self.assertEqual(b.cached['sweep']['required_mm']['table'],10.)
        scene['table_estimate']['top_m']=-.045
        with self.assertRaises(ValueError):
            b.recheck_final_scene(state,scene,dict(image_time=time.time()),at_entry=False,corner_change=2.,remaining=.5)
    def test_already_horizontal_at_high_waypoint_proceeds_to_entry_not_repeat_high(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from visual_docking_live import requires_high_stage
        high=np.eye(4);current=high.copy();current[2,3]=.008
        current[:3,:3]=Rotation.from_euler('y',1,degrees=True).as_matrix()
        import json
        result=requires_high_stage(current,high)
        self.assertFalse(result)
        self.assertEqual(json.loads(json.dumps(dict(required=result))),dict(required=False))
        current[2,3]=.03
        self.assertTrue(requires_high_stage(current,high))
        current=high.copy();current[:3,:3]=Rotation.from_euler('x',90,degrees=True).as_matrix()
        self.assertTrue(requires_high_stage(current,high))
        current=high.copy();current[:3,3]=[.045,0.,-.045]
        self.assertFalse(requires_high_stage(current,high),'Already leveled near entry must not repeat high preparation')
    def test_task_snapshot_shows_latest_unknown_grip_not_old_saved_stability(self):
        import threading
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            t=LiveDockTask(None,None,directory=directory,auto_worker=False)
            t.state['grip']=dict(stable=True)
            t.backend=SimpleNamespace(grip_lock=threading.RLock(),grip=dict(stable=False,reason='ambiguous'),grip_stamp=12.)
            s=t.snapshot()
            self.assertFalse(s['grip']['stable'])
            self.assertEqual(s['grip_captured_at'],12.)
    def test_outside_coarse_knots_preserve_endpoint_with_same_timing_limits(self):
        import numpy as np,json
        from handeye_column import ColumnModel
        from agent.grasp_path import plan_correction
        root=Path(__file__).resolve().parents[2];m=ColumnModel(side='right');q=np.zeros(6);h=5.4
        tcp=np.array(json.loads((root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool']);end=q.copy();end[5]=-.1745329252
        target=m.wrist_transform(end,h)@tcp;limits=json.loads((root/'web/config/control.json').read_text())['arms']['right']['joint_limits_deg']
        timing=dict(joint_speed_deg_s=[3.]*6,joint_acceleration_deg_s2=[6.]*6,tcp_speed_mm_s=12.,tcp_acceleration_mm_s2=25.)
        fine=plan_correction(m,q,h,tcp,target,limits,timing_limits=timing)
        coarse=plan_correction(m,q,h,tcp,target,limits,timing_limits=timing,translation_step_mm=5.,rotation_step_deg=2.)
        self.assertLess(len(coarse['joint_waypoints_rad']),len(fine['joint_waypoints_rad']))
        self.assertTrue(np.allclose(m.wrist_transform(coarse['joint_waypoints_rad'][-1],h)@tcp,target,atol=1e-5))
        self.assertEqual(coarse['timing']['limits'],fine['timing']['limits'])
    def test_level_first_path_turns_outside_then_raises_before_approaching(self):
        import numpy as np,json
        from handeye_column import ColumnModel
        root=Path(__file__).resolve().parents[2]
        m=ColumnModel(side='right');q=np.zeros(6);h=5.412
        tcp=np.array(json.loads((root/'config/right-grip-tcp.json').read_text())['T_wrist_from_tool'])
        start=m.wrist_transform(q,h)@tcp;target=np.eye(4)
        a=.1;target[:3,:3]=[[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]]
        target[:3,3]=[.46,-.10,.665]
        limits=json.loads((root/'web/config/control.json').read_text())['arms']['right']['joint_limits_deg']
        timing=dict(joint_speed_deg_s=[3.]*6,joint_acceleration_deg_s2=[6.]*6,tcp_speed_mm_s=12.,tcp_acceleration_mm_s2=25.)
        p=level_first_path(m,q,h,tcp,target,limits,timing)
        self.assertEqual([v['kind'] for v in p['legs']],['roll_j6_outside','level_outside','raise_outside','approach_high'])
        boundary=p['legs'][0]['end_waypoint']
        first=np.asarray(p['joint_waypoints_rad'][:boundary+1])
        self.assertTrue(np.allclose(first[:,:5],q[:5]))
        self.assertAlmostEqual(first[-1,5],-np.pi/2)
        self.assertAlmostEqual(p['legs'][2]['target'][2][3],target[2,3])
        final=m.wrist_transform(p['joint_waypoints_rad'][-1],h)@tcp
        self.assertLess(np.linalg.norm(final[:3,3]-target[:3,3]),.001)
    def test_cabinet_drift_budget_does_not_consume_robot_relative_clearance(self):
        sweep=dict(lower_mm=dict(cabinet_standoff=55.,table=18.,base=20.1,left_arm=20.1,own_arm=10.1))
        self.assertEqual(scene_drift_allowance(sweep),5.)
        sweep['lower_mm']['cabinet_standoff']=51.
        self.assertEqual(scene_drift_allowance(sweep),1.)
    def test_grip_watcher_never_takes_controller_lock_while_holding_grip_lock(self):
        import json,time,cv2,threading
        from types import SimpleNamespace
        from agent.docking_grip import GripTracker
        from agent.tests.test_visual_docking import image
        class Lock:
            held=False
            def __enter__(self):self.held=True
            def __exit__(self,*a):self.held=False
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);p=root/'web/public/camera-live';p.mkdir(parents=True)
            now=time.time();name='d405-v-a-1.jpg';cv2.imwrite(str(p/name),image()[:,:,::-1])
            (p/'d405.json').write_text(json.dumps(dict(state='online',captured_at=now,raw_image_file=name,color_intrinsics=dict(serial='test'))))
            b=LiveDockBackend.__new__(LiveDockBackend);b.root=root;b.serial='test';b.tracker=GripTracker();b.grip=None;b.grip_stamp=-1.;b.grip_lock=Lock();b.cancel_event=threading.Event()
            def snapshot():
                self.assertFalse(b.grip_lock.held,'ABBA: controller lock requested inside grip lock')
                return dict(arms=dict(right=dict(timestamp=now,gripper=.88)))
            b.ctrl=SimpleNamespace(snapshot=snapshot)
            with self.assertRaises(ValueError):b._grip(settle=False)
    def test_stationary_short_of_insertion_cannot_be_confirmed(self):
        import numpy as np
        with self.assertRaises(ValueError):validate_arrival('insert',np.eye(4),np.eye(4),dict(leading_standoff_mm=20.))
        validate_arrival('insert',np.eye(4),np.eye(4),dict(leading_standoff_mm=-5.,nominal_clearance_mm=1.))
    def test_withdrawal_must_recover_original_standoff(self):
        import numpy as np
        with self.assertRaises(ValueError):validate_arrival('withdraw',np.eye(4),np.eye(4),dict(leading_standoff_mm=35.),start_standoff=55.)
    def test_lift_stop_remains_available_during_uncertain_goal_ack_and_refences_late_goal(self):
        from types import SimpleNamespace
        class Future:
            def __init__(self):self.ready=False;self.callbacks=[]
            def done(self):return self.ready
            def add_done_callback(self,f):self.callbacks.append(f)
            def result(self):return SimpleNamespace(success=True,message='held')
            def complete(self):
                self.ready=True
                for f in list(self.callbacks):f(self)
        calls=[]
        def send(_):f=Future();calls.append(f);return f
        a=RosAdapter({});goal=Future();a.pending_lift=goal
        a.stop_lift_client=SimpleNamespace(call_async=send);a.Trigger=SimpleNamespace(Request=lambda:object())
        a.controller=SimpleNamespace(log=lambda *a:None)
        a.lift_stop();self.assertEqual(len(calls),1);self.assertIs(a.pending_lift,goal)
        goal.complete();calls[0].complete()
        self.assertEqual(len(calls),2)
    def test_optional_driver_reference_preserved_for_load_compensation(self):
        a=RosAdapter({});a.node=object();a.update_trajectory('right',status())
        self.assertEqual(a.trajectory_status('right')['joint_reference_rad'],[0.]*6)
        self.assertEqual(a.trajectory_status('right')['gripper_target_raw'],0.)
    def test_malformed_reference_invalidates_receipt(self):
        a=RosAdapter({});a.node=object();s=status();s['joint_reference_rad'][2]=float('nan')
        a.update_trajectory('right',s);self.assertIsNone(a.trajectory_status('right'))
    def test_segments_preserve_exact_curve_and_stop_within_driver_time_limit(self):
        points=[[i*.001]*6 for i in range(91)];times=list(range(91))
        pieces=split_plan(points,times)
        self.assertTrue(all(p['time_from_start_s'][-1]<=45 for p in pieces))
        self.assertEqual(pieces[0]['joint_waypoints_rad'][0],points[0]);self.assertEqual(pieces[-1]['joint_waypoints_rad'][-1],points[-1])
        self.assertEqual(pieces[0]['joint_waypoints_rad'][-1],pieces[1]['joint_waypoints_rad'][0])
    def test_browser_cannot_select_release_targets_or_gains(self):
        with tempfile.TemporaryDirectory() as d:
            t=LiveDockTask(None,None,directory=Path(d),auto_worker=False)
            for payload in ({'op':'release'},{'op':'start','model':'demo','joints':[0]*6},{'op':'start','model':'demo','gain':1}):
                with self.assertRaises(ValueError):t.action(payload)
    def test_existing_recovery_receipt_cannot_autostart(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'receipt.json').write_text(json.dumps(dict(status='running')))
            t=LiveDockTask(None,None,directory=p,auto_worker=False)
            self.assertEqual(t.snapshot()['status'],'recovery_required')
            with self.assertRaises(ValueError):t.action(dict(op='start',model='demo'))
    def test_live_http_endpoint_preserves_cookie_origin_and_target_allowlist(self):
        import http.client,json,threading
        from http.server import ThreadingHTTPServer
        from server import make_handler
        from control import Controller
        from test_control import FakeAdapter,configured
        from test_cabinet_http import Cabinet,Execution
        with tempfile.TemporaryDirectory() as d:
            adapter=FakeAdapter();ctrl=Controller(configured(),adapter)
            task=LiveDockTask(ctrl,Cabinet(),directory=Path(d),auto_worker=False)
            server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(ctrl,cabinet_preview=Cabinet(),cabinet_execution=Execution(),visual_docking=task))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            def call(method,path,data=None,headers=None):
                c=http.client.HTTPConnection('127.0.0.1',server.server_port)
                c.request(method,path,None if data is None else json.dumps(data),headers or {})
                r=c.getresponse();answer=r.status,dict(r.getheaders()),r.read();c.close();return answer
            try:
                self.assertEqual(call('POST','/api/visual-docking',dict(op='start',model='demo'))[0],403)
                _,h,_=call('GET','/visual-docking.html')
                headers={'Cookie':h['Set-Cookie'].split(';')[0],'Origin':'http://127.0.0.1:'+str(server.server_port),'Content-Type':'application/json'}
                self.assertEqual(call('GET','/api/visual-docking',headers=headers)[0],200)
                self.assertEqual(call('POST','/api/visual-docking',dict(op='start',model='demo',joints=[0]*6),headers)[0],400)
                self.assertEqual(call('POST','/api/visual-docking',dict(op='start',model='demo'),headers)[0],200)
                self.assertEqual(adapter.calls,[])
            finally:server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
