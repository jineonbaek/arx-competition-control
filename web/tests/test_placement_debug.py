from pathlib import Path
import tempfile
import time
import unittest

from placement_debug import PlacementDebug
from placement_reference_store import save_reference_record,select_active_reference

ROOT=Path(__file__).resolve().parents[2]

class Controller:
    def __init__(self):self.token=None;self.epoch=1;self.reserved=0;self.released=0;self.busy=False
    def snapshot(self):return dict(command_epoch=self.epoch,motion_reserved=self.token is not None,
        competition_reserved=self.busy,chassis_active=False,body={'height':0.,'fresh':True})
    def reserve_motion(self,epoch):
        if epoch!=self.epoch:raise ValueError('epoch changed')
        self.token=object();self.reserved+=1;return self.token
    def release_motion(self,token):
        if token is not self.token:raise ValueError('wrong token')
        self.token=None;self.released+=1
    def motion_valid(self,token):return token is not None and token is self.token

class Cabinet:
    def __init__(self):self.calls=[];self.offset=0.
    def action(self,data,*,include_arm_preview,token):
        self.calls.append((data,token))
        return dict(status='localized',image_time=time.time(),image='test-image',
            window={'center_base_m':[.5+self.offset,0.,.5],'inside_axis_base':[1.,0.,0.],'pose_ambiguous':False})

class Arm:
    thread=None
    def __init__(self):self.starts=[];self.cancelled=0;self.status='complete'
    def start_owned(self,token,handoff):self.starts.append((token,handoff));return {'run_id':'arm-debug'}
    def snapshot(self):return dict(run_id='arm-debug',status=self.status,released=True,depth_mm=150.,insertion_depth_verified=True,verified_depth_mm=150.)
    def action(self,data):self.cancelled+=1

class PlacementDebugTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        (self.root/'config').mkdir()
        for name in ('taught-placement.json','visual-chassis-placement.json'):
            (self.root/'config'/name).write_bytes((ROOT/'config'/name).read_bytes())
        self.ctrl=Controller();self.cabinet=Cabinet();self.arm=Arm()
        self.debug=PlacementDebug(self.ctrl,self.cabinet,self.arm,self.root)
        path=self.root/'web/hardware/runtime/placement-references'
        record=save_reference_record(path,{'d435':{'valid':True,'captured_at':time.time()-100.,
                'center_base_m':[.5,0.,.5],'inside_axis_base':[1.,0.,0.]}})
        select_active_reference(path,record['reference_id'])

    def wait(self):
        self.debug.thread.join(2.);self.assertFalse(self.debug.thread.is_alive());return self.debug.snapshot()

    def place(self,request='place-1'):
        return dict(op='place',request_id=request,expected_epoch=1,environment_clear=True,base_fixed_confirmed=True)

    def test_observe_finishes_without_ownership_arm_or_navigation(self):
        self.debug.action('browser',{'op':'observe','request_id':'observe-1'});result=self.wait()
        self.assertEqual(result['status'],'observed');self.assertTrue(result['aligned'])
        self.assertEqual(self.arm.starts,[]);self.assertEqual(self.ctrl.reserved,0)
        self.assertFalse(result['sends_navigation_receipt']);self.assertFalse(result['moves_chassis'])

    def test_standalone_place_works_without_any_navigation_session(self):
        self.debug.action('browser',self.place());result=self.wait()
        self.assertEqual(result['status'],'completed');self.assertEqual(len(self.arm.starts),1)
        self.assertEqual(self.ctrl.reserved,1);self.assertEqual(self.ctrl.released,1)
        self.assertEqual(self.arm.starts[0][1]['chassis_pose']['source'],'operator_confirmed_fixed_base')
        self.assertFalse(result['sends_navigation_receipt']);self.assertFalse(result['control_retained'])

    def test_standalone_place_accepts_height_above_old_window(self):
        snapshot=self.ctrl.snapshot
        def actual_height():
            value=snapshot();value['body']['height']=.18;return value
        self.ctrl.snapshot=actual_height
        self.debug.action('browser',self.place());result=self.wait()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(len(self.arm.starts),1)

    def test_observe_does_not_automatically_start_place(self):
        self.debug.action('browser',{'op':'observe','request_id':'observe-1'});self.wait()
        self.assertEqual(self.arm.starts,[])
        self.debug.action('browser',self.place());self.wait();self.assertEqual(len(self.arm.starts),1)

    def test_bad_alignment_never_moves_chassis_or_starts_arm(self):
        self.cabinet.offset=.10;self.debug.action('browser',self.place());result=self.wait()
        self.assertEqual(result['status'],'failed');self.assertEqual(self.arm.starts,[])
        self.assertEqual(self.ctrl.released,1)

    def test_competition_ownership_cannot_be_bypassed_in_debug(self):
        self.ctrl.busy=True
        with self.assertRaises(ValueError):self.debug.action('browser',self.place())
        self.assertEqual(self.ctrl.reserved,0);self.assertEqual(self.arm.starts,[])

    def test_duplicate_place_request_cannot_replay_push_or_release(self):
        request=self.place();self.debug.action('browser',request);self.wait()
        self.debug.action('browser',request);self.assertEqual(len(self.arm.starts),1)

    def test_confirmation_is_required_before_any_ownership_or_arm_action(self):
        request=self.place();request['base_fixed_confirmed']=False
        with self.assertRaises(ValueError):self.debug.action('browser',request)
        self.assertEqual(self.ctrl.reserved,0)

    def test_stale_cancel_request_does_not_cancel_current_arm(self):
        self.debug.action('browser',{'op':'observe','request_id':'observe-1'});self.wait()
        with self.assertRaises(ValueError):self.debug.action('browser',{'op':'cancel','request_id':'old-run'})
        self.assertEqual(self.arm.cancelled,0)

    def test_unfinished_arm_cleanup_retains_control_and_prevents_next_run(self):
        class Unfinished:
            def join(self,timeout):pass
            def is_alive(self):return True
        self.arm.thread=Unfinished()
        self.debug.action('browser',self.place());result=self.wait()
        self.assertEqual(result['status'],'failed');self.assertTrue(result['control_retained'])
        self.assertEqual(self.ctrl.released,0);self.assertFalse(result['can_start'])
        with self.assertRaises(ValueError):self.debug.action('browser',self.place('another-run'))

    def test_failed_stop_still_checks_worker_and_retains_control(self):
        class Unfinished:
            def join(self,timeout):pass
            def is_alive(self):return True
        self.arm.thread=Unfinished()
        def fail_cancel(data):raise ValueError('stop rejected')
        self.arm.action=fail_cancel
        self.debug.action('browser',self.place());result=self.wait()
        self.assertTrue(result['control_retained']);self.assertEqual(self.ctrl.released,0)
        self.assertEqual(result['cleanup_reason'],'stop rejected');self.assertFalse(result['can_start'])

if __name__=='__main__':unittest.main()
