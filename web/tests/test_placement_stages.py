import copy,json,tempfile,time,unittest
from pathlib import Path
import numpy as np
from unittest.mock import patch
from placement_stages import PlacementStages

class Model:
    reach=.7
    def wrist_transform(self,q,h):
        T=np.eye(4);T[:3,3]=np.asarray(q[:3])/1000+[.4,-.2,.5+h/41.54];return T
    def check(self,a,b,h):return {'clear':True,'lower_mm':50.}

class Controller:
    def __init__(self):
        self.token=None;self.epoch=10;self.calls=[];self.active=False;self.seq=1
        self.target=-1.247;self.q=[0.]*6;self.config={'arms':{'right':{'gripper_closed_raw':0,'trajectory_policy':{'speed_deg_s':50,'acceleration_deg_s2':18},'joint_limits_deg':[[-100.,100.]]*6}}};self.models={'right':Model()};self.adapter=self
    def snapshot(self):return dict(command_epoch=self.epoch,motion_reserved=self.token is not None,competition_reserved=False,
        chassis_active=self.active,chassis_scheduler={'output_seq':self.seq},
        arms={s:dict(joints=list(self.q),gripper=-.5,fresh=True,received=time.monotonic()) for s in ('left','right')},body={'height':.2,'fresh':True})
    def trajectory_status(self,side):return dict(driver_instance='test',command_epoch=1,ready=True,enabled=True,gripper_target_raw=self.target,joint_reference_rad=self.q,status='idle')
    def submit(self,command,token,*,preflight):
        assert self.motion_valid(token) and preflight(self.snapshot(),None)
        self.target=command['position'];self.calls.append(command);return {'ok':True}
    def reserve_motion(self,epoch):
        if epoch!=self.epoch:raise ValueError('epoch changed')
        self.token=object();self.calls.append('reserve');return self.token
    def motion_valid(self,token):return token is not None and token is self.token
    def release_motion(self,token):self.token=None;self.calls.append('release')
    def clock(self):return time.monotonic()

class Capture:
    def __init__(self):self.calls=[]
    def action(self,data):self.calls.append(data);raise ValueError('no fixture camera')
class Push:
    def __init__(self):self.calls=[];self.state={}
    def action(self,data):self.calls.append(data);self.state=dict(status='completed_travel',request_id=data['request_id'],measured_travel_mm=160.,ended_at=time.time());return self.state
    def snapshot(self):return self.state

class StageTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        (self.root/'config').mkdir();(self.root/'config/right-grip-tcp.json').write_text(json.dumps({'T_wrist_from_tool':np.eye(4).tolist()}))
        self.ctrl=Controller();self.capture=Capture();self.push=Push()
        self.task=PlacementStages(self.ctrl,self.capture,self.push,self.root);self.addCleanup(self.task.close)

    def test_get_and_edit_or_record_preobserve_never_move_or_capture(self):
        s=self.task.snapshot();self.assertEqual(s['settings']['cabinet_z_mm'],630.)
        self.assertIsNone(s['height']['tcp_z_mm']);self.assertAlmostEqual(s['height']['model_tcp_z_mm'],504.8146,places=3)
        self.task.action({'op':'record_preobserve'})
        self.assertEqual(self.task.snapshot()['settings']['preobserve_joints_deg'],[0.]*6)
        settings=self.task.snapshot()['settings'];settings['target_z_mm']=635.
        self.task.action({'op':'save_settings','settings':settings})
        self.assertEqual(self.ctrl.calls,[]);self.assertEqual(self.capture.calls,[])
        other=PlacementStages(self.ctrl,self.capture,self.push,self.root)
        self.addCleanup(other.close);self.assertEqual(other.snapshot()['settings']['target_z_mm'],635.)

    def test_rejects_invalid_settings_without_overwriting_point(self):
        old=self.task.snapshot()['settings'];bad=copy.deepcopy(old);bad['preobserve_joints_deg'][0]=999.
        with self.assertRaises(ValueError):self.task.action({'op':'save_settings','settings':bad})

    def test_failed_save_does_not_change_live_parameters(self):
        old=self.task.snapshot()['settings'];new=copy.deepcopy(old);new['target_z_mm']=700.
        with patch('placement_stages.atomic_json',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):self.task.action({'op':'save_settings','settings':new})
        self.assertEqual(self.task.snapshot()['settings'],old)
        self.assertEqual(self.task.snapshot()['settings'],old)
        bad=copy.deepcopy(old);bad['cabinet_z_mm']=float('nan')
        with self.assertRaises(ValueError):self.task.action({'op':'save_settings','settings':bad})

    def test_preobserve_uses_project_travel_timing_and_splits_long_paths(self):
        from placement_stages import time_preobserve, split_plan
        from agent.grasp_path import TimedJointPath
        class TravelModel(Model):
            def wrist_transform(self,q,h):
                T=np.eye(4);T[0,3]=.6*q[0];return T
        model=TravelModel();tcp=np.eye(4);q=np.zeros(6)
        limits=dict(joint_speed_deg_s=[15.]*3+[50.]*3,joint_acceleration_deg_s2=[18.]*6,
                    tcp_speed_mm_s=60.,tcp_acceleration_mm_s2=120.)
        for distance in (.8,2.8):
            goal=np.array([distance,0.,0.,0.,0.,0.])
            path=time_preobserve(q,goal,model,.4,tcp,limits,lambda:None)
            segments=split_plan(path.q.tolist(),path.times.tolist())
            np.testing.assert_allclose(path.q[[0,-1]],[q,goal])
            self.assertTrue(all(0<part['time_from_start_s'][-1]<=45. for part in segments))
            self.assertEqual(len(segments)>1,distance==2.8)
            for part in segments:
                timing=TimedJointPath(part['joint_waypoints_rad'],model.reach,
                    tcp_position=lambda v:model.wrist_transform(v,.4)[:3,3],**limits)
                np.testing.assert_allclose(timing.times,part['time_from_start_s'])

    def test_preobserve_replaces_aperture_hold_with_zero_and_keeps_it(self):
        config=Path(__file__).resolve().parents[2]/'config/battery-execution.json'
        (self.root/'config/battery-execution.json').write_bytes(config.read_bytes())
        for request in ('first','already-closed'):
            self.task.action(dict(op='run',phase='preobserve',expected_epoch=10,request_id=request))
            self.task.thread.join(3)
            result=self.task.snapshot()
            self.assertEqual(result['status'],'completed',result.get('reason'))
            self.assertEqual(result['clamp']['target_raw'],0)
            self.assertEqual(result['clamp']['feedback_raw'],-.5)
            self.assertEqual(result['clamp']['close_command_sent'],request=='first')
        self.assertEqual([cmd for cmd in self.ctrl.calls if isinstance(cmd,dict)],
                         [dict(type='gripper',arm='right',position=0)])

    def test_correction_stops_if_driver_loses_close_target_during_observation(self):
        config=Path(__file__).resolve().parents[2]/'config/battery-execution.json'
        (self.root/'config/battery-execution.json').write_bytes(config.read_bytes())
        def change_target():
            self.ctrl.target=-1.247
            return {},{}
        with patch.object(self.task,'_capture',side_effect=change_target):
            self.task.action(dict(op='run',phase='correct',expected_epoch=10,request_id='lost'))
            self.task.thread.join(3)
        self.assertIn('闭合目标',self.task.snapshot()['reason'])
        self.assertEqual(self.push.calls,[])

    def test_rejected_closure_prevents_preobserve_motion(self):
        with patch.object(self.ctrl,'submit',return_value={'ok':False}), patch('placement_stages.DockingMotion') as motion:
            self.task.action(dict(op='run',phase='preobserve',expected_epoch=10,request_id='refused'))
            self.task.thread.join(2)
            self.assertEqual(self.task.snapshot()['status'],'blocked')
            motion.assert_not_called()

    def test_manual_height_anchor_is_invalid_after_chassis_motion(self):
        self.task.action({'op':'calibrate_z','known_tcp_z_mm':630.})
        s=self.task.snapshot();self.assertAlmostEqual(s['height']['tcp_z_mm'],630.)
        self.ctrl.q[2]+=10.;self.assertAlmostEqual(self.task.snapshot()['height']['tcp_z_mm'],640.)
        self.ctrl.seq=2;self.assertIsNone(self.task.snapshot()['height']['tcp_z_mm'])

    def test_insert_runs_without_preobserve_or_correct_and_does_not_chain(self):
        settings=self.task.snapshot()['settings'];settings['z_reference']='model'
        self.task.action({'op':'save_settings','settings':settings})
        data={'op':'run','phase':'insert','request_id':'one','expected_epoch':10}
        self.task.action(data);self.task.thread.join(2.)
        self.assertEqual(self.task.snapshot()['status'],'completed_travel')
        self.assertEqual(len(self.push.calls),1);self.assertEqual(self.ctrl.calls,[])
        self.task.action(data);self.assertEqual(len(self.push.calls),1)
        self.assertEqual(self.task.snapshot()['operation'],'insert')

    def test_wrapper_failure_cancels_only_its_delegated_insert(self):
        settings=self.task.snapshot()['settings'];settings['z_reference']='model'
        self.task.action({'op':'save_settings','settings':settings})
        push=self.push;original=push.snapshot;pending=[True]
        def snapshot():
            if pending[0] and push.calls:
                pending[0]=False;raise ValueError('poll failed')
            return original()
        push.snapshot=snapshot
        self.task.action({'op':'run','phase':'insert','request_id':'owned','expected_epoch':10})
        self.task.thread.join(2.)
        self.assertEqual(push.calls[-1],{'op':'cancel','request_id':'owned'})

    def test_correction_stops_on_no_residual_improvement_without_attempt_limit(self):
        from placement_stages import require_residual_improvement,check_tool_arrival
        with self.assertRaises(ValueError):require_residual_improvement(2.,2.,.5,.5)
        require_residual_improvement(2.,1.7,.5,.5)
        T=np.eye(4);target=T.copy();target[2,3]=.0005
        with self.assertRaises(ValueError):check_tool_arrival(T,T,target)

if __name__=='__main__':unittest.main()
