import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from navigation_placement import NavigationPlacement
from placement_reference_store import save_reference_record,select_active_reference

ROOT=Path(__file__).resolve().parents[2]

class Controller:
    def __init__(self):self.token=object();self.ended=[];self.height=0.
    def motion_valid(self,token):return token is self.token
    def end_chassis_output(self,token):self.ended.append(token)
    def snapshot(self):return {'chassis_active':False,'body':{'height':self.height,'fresh':True}}

class Navigation:
    active=True;mission='mission-1'
    def __init__(self):
        self.visit={'visit_id':'visit-1','point_id':'point_2','mission_id':self.mission};self.events=[]
    def check(self):return None
    def _event(self,event,**data):self.events.append((event,data))

class Placement:
    thread=None
    def __init__(self,status='complete'):self.status=status;self.starts=[];self.actions=[]
    def start_owned(self,token,handoff):
        self.starts.append((token,handoff));return {'run_id':'arm-1'}
    def snapshot(self):return dict(status=self.status,run_id='arm-1',released=True,returned=True,handle_outside_mm=45.,depth_mm=150.,insertion_depth_verified=True,verified_depth_mm=150.,reason='injected fault')
    def action(self,data):self.actions.append(data)

class Module(NavigationPlacement):
    def _observe(self,context,run):
        return {'d435':{'valid':True,'captured_at':time.time(),
                        'center_base_m':[.5,0.,.5],'inside_axis_base':[1.,0.,0.]}}
    def _stop_chassis(self,context,run):return {'stopped':True,'pose':{'x':0.,'y':0.,'yaw':0.}}
    def _chassis_step(self,step,context,run):raise AssertionError('Aligned fixture must not command chassis')

class NavigationPlacementTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);(self.root/'config').mkdir()
        (self.root/'config/visual-chassis-placement.json').write_bytes((ROOT/'config/visual-chassis-placement.json').read_bytes())
        (self.root/'config/taught-placement.json').write_bytes((ROOT/'config/taught-placement.json').read_bytes())
        self.ctrl=Controller();self.nav=Navigation();self.arm=Placement()
        self.module=Module(self.ctrl,None,self.arm,self.nav,self.root)
        self.context=dict(request_id='run:place',token=self.ctrl.token,cancelled=threading.Event())
        self.record(True)

    def record(self,valid):
        r=save_reference_record(self.module.reference_root,{'d435':{'valid':valid,'captured_at':time.time()-100.,
             'center_base_m':[.5,0.,.5],'inside_axis_base':[1.,0.,0.]}})
        select_active_reference(self.module.reference_root,r['reference_id'])

    def wait(self):
        self.module.runs[self.context['request_id']]['thread'].join(2.)
        self.assertFalse(self.module.runs[self.context['request_id']]['thread'].is_alive())
        return self.module.inspect(self.context)

    def test_current_position_test_has_no_lidar_visit_or_navigation_events(self):
        from navigation_placement import CurrentPositionPlacement
        self.module=CurrentPositionPlacement(self.ctrl,self.arm,self.root)
        self.context.update(route_scope='t2_test',run_id='run')
        self.module.start(self.context)
        result=self.wait()
        self.assertEqual(result['status'],'completed',result)
        handoff=self.arm.starts[0][1]
        self.assertEqual(handoff['admission_source'],'t2_current_position')
        self.assertNotIn('navigation_visit_id',handoff)
        self.assertNotIn('point_id',handoff)
        with self.assertRaises(ValueError):
            self.module.start(self.context)
        self.assertEqual(len(self.arm.starts),1)

    def test_current_position_requires_ownership_stationary_chassis_and_test_scope(self):
        from navigation_placement import CurrentPositionPlacement
        self.module=CurrentPositionPlacement(self.ctrl,self.arm,self.root)
        with self.assertRaises(ValueError):
            self.module.start(self.context)
        self.context.update(route_scope='t2_test',run_id='run')
        with self.assertRaises(ValueError):
            self.module.start(self.context | dict(token=object()))
        self.ctrl.snapshot=lambda:dict(chassis_active=True)
        with self.assertRaises(ValueError):
            self.module.start(self.context)
        self.assertFalse(self.arm.starts)

    def test_point_two_admission_uses_existing_token_and_waits_for_full_return_receipt(self):
        self.module.start(self.context);result=self.wait()
        self.assertEqual(result['status'],'completed')
        self.assertEqual(result['phase'],'returned')
        token,handoff=self.arm.starts[0]
        self.assertIs(token,self.ctrl.token)
        self.assertEqual(handoff['navigation_visit_id'],'visit-1')
        self.assertEqual(handoff['outcome'],'locked_admitted')
        saved=json.loads((self.root/'reports/visual-chassis-active/handoff.json').read_text())
        self.assertEqual(saved['navigation_visit_id'],'visit-1')
        self.assertEqual(self.nav.events,[]) # Competition releases ownership before complete_route sends complete.

    def test_unverified_old_receipt_cannot_complete_navigation_visit(self):
        original=self.arm.snapshot
        self.arm.snapshot=lambda:original()|dict(status='complete_unverified',returned=True,insertion_depth_verified=False,verified_depth_mm=140.)
        self.module.start(self.context);result=self.wait()
        self.assertEqual(result['status'],'failed')
        self.assertEqual([name for name,_ in self.nav.events],['fail'])

    def test_arm_travel_without_visual_depth_is_not_success(self):
        original=self.arm.snapshot
        self.arm.snapshot=lambda:original()|dict(insertion_depth_verified=False)
        self.module.start(self.context);result=self.wait()
        self.assertEqual(result['status'],'failed')

    def test_duplicate_request_cannot_replay_insertion_or_release(self):
        self.module.start(self.context);self.wait()
        with self.assertRaises(ValueError):self.module.start(self.context)
        self.assertEqual(len(self.arm.starts),1)

    def test_new_request_for_consumed_navigation_visit_cannot_replay(self):
        self.module.start(self.context);self.wait()
        with self.assertRaisesRegex(ValueError,'导航到访'):
            self.module.start(self.context|dict(request_id='different-request'))
        self.assertEqual(len(self.arm.starts),1)

    def test_placement_does_not_gate_on_lift_reference_difference(self):
        self.ctrl.height=3.;self.module.start(self.context);result=self.wait()
        self.assertEqual(result['status'],'completed')
        self.assertEqual(len(self.arm.starts),1)

    def test_invalid_reference_reports_failure_without_starting_arm(self):
        self.record(False);self.module.start(self.context);result=self.wait()
        self.assertEqual(result['status'],'failed')
        self.assertEqual(self.arm.starts,[])
        self.assertEqual(self.nav.events[0][0],'fail')
        self.assertFalse(self.module.readiness()['reference_valid'])

    def test_wrong_arrival_point_cannot_start_placement(self):
        self.nav.visit['point_id']='point_1'
        with self.assertRaises(ValueError):self.module.start(self.context)
        self.assertEqual(self.arm.starts,[])

    def test_degraded_arm_receipt_is_not_completed_placement(self):
        self.arm.status='complete_degraded';self.module.start(self.context);result=self.wait()
        self.assertEqual(result['status'],'failed')
        self.assertEqual(self.nav.events[0][0],'fail')

    def test_invalid_control_ownership_is_rejected(self):
        self.context['token']=object()
        with self.assertRaises(ValueError):self.module.start(self.context)
        self.assertEqual(self.arm.starts,[])

    def test_stale_stop_cannot_cancel_a_new_arm_run(self):
        self.module.start(self.context);self.wait()
        self.arm.snapshot=lambda:dict(run_id='a-different-run',status='running')
        result=self.module.stop(self.context)
        self.assertTrue(result['stopped'])
        self.assertEqual(self.arm.actions,[])

if __name__=='__main__':unittest.main()
