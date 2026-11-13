"""Dynamic Agent request routing and recovery with an inert prepared step."""
import copy
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from web.src.handeye_column import ColumnModel
from agent.battery_flow import plan_digest
from agent.storage import RunStore
from agent.battery_backend import BatteryControllerBackend
from agent.tests.test_coarse_backend import Step
from agent.tests.test_execution_policy import policy

class Controller:
    def motion_valid(self,token):return token=='owned'

class BackendTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.store=RunStore.create(self.tmp.name);self.addCleanup(self.store.close)
        self.c=Controller();self.step=Step();self.op='execute_coarse_alignment'
        self.request=self.store.state['run_id']+':coarse'
        self.payload={'plan_id':'p','plan':{'plan_id':'p','arm':'right'}}
        self.store.state['mission']['plan']=copy.deepcopy(self.payload['plan'])
        self.pending();self.b=self.make()
    def make(self):
        b=BatteryControllerBackend(self.c,'owned',self.store,policies={},pick_setup=None,
            preflight=lambda *a:True,cancelled=lambda:False,timeout_s={
                'execute_coarse_alignment':1.,'execute_battery_approach':1.,'execute_pick':1.})
        self.addCleanup(lambda:b.cancel_and_wait(2))
        return b
    def pending(self):
        self.store.state['pending']=dict(operation=self.op,request_id=self.request,arguments=copy.deepcopy(self.payload))
        self.store.save()
    def start(self):return self.b.invoke(self.op,self.payload,self.request)
    def test_request_is_prepared_once_and_receipt_survives_reopen(self):
        with patch('agent.battery_backend.prepare_step',return_value=self.step) as prepare:
            self.start();self.assertTrue(self.step.seen.wait(1));self.start()
            prepare.assert_called_once()
            self.step.finish.set();self.assertTrue(self.b.entries[self.request].wait(1))
            self.assertEqual(self.b.status(self.op,self.request)['status'],'completed')
            recovered=self.make()
            self.assertEqual(recovered.invoke(self.op,self.payload,self.request)['status'],'completed')
            prepare.assert_called_once()
        self.assertEqual(self.step.calls,['start'])
    def test_uncertain_journal_never_prepares_or_replays(self):
        with patch('agent.battery_backend.prepare_step',return_value=self.step) as prepare:
            self.start();self.assertTrue(self.step.seen.wait(1))
            recovered=self.make();r=recovered.invoke(self.op,self.payload,self.request)
            self.assertEqual(r['status'],'unknown');prepare.assert_called_once()
            self.assertEqual(self.step.calls,['start'])
    def test_payload_pending_identity_and_mission_plan_cannot_be_replaced(self):
        with patch('agent.battery_backend.prepare_step') as prepare:
            self.store.state['pending']=None
            self.assertEqual(self.start()['status'],'rejected')
            self.pending();self.store.state['mission']['plan']['arm']='left'
            self.assertEqual(self.start()['status'],'rejected')
            self.assertEqual(self.b.invoke(self.op,self.payload,'other:request')['status'],'rejected')
            prepare.assert_not_called()
    def test_same_request_with_changed_payload_is_rejected(self):
        with patch('agent.battery_backend.prepare_step',return_value=self.step):
            self.start();self.assertTrue(self.step.seen.wait(1))
            changed=copy.deepcopy(self.payload);changed['plan_id']='different'
            self.assertEqual(self.b.invoke(self.op,changed,self.request)['status'],'rejected')
            self.assertEqual(self.make().invoke(self.op,changed,self.request)['status'],'rejected')
    def test_missing_tcp_capability_and_failed_preparation_have_no_effects(self):
        self.assertTrue(self.b.available(self.op))
        self.assertFalse(self.b.available('execute_pick'))
        self.assertFalse(self.b.available('execute_battery_approach'))
        with patch('agent.battery_backend.prepare_step',side_effect=ValueError('No complete clearance')):
            r=self.start()
        self.assertEqual(r['status'],'rejected');self.assertIn('clearance',r['data']['reason'])
        self.assertEqual(self.step.calls,[])
        self.assertEqual(list(self.b.directory.glob('*.json')),[])
    def test_cancel_is_not_a_successful_grasp(self):
        with patch('agent.battery_backend.prepare_step',return_value=self.step):
            self.start();self.assertTrue(self.step.seen.wait(1))
            self.assertTrue(self.b.cancel_and_wait(1))
            r=self.b.status(self.op,self.request)
            self.assertEqual(r['status'],'unknown');self.assertFalse(r['data']['grasp_confirmed'])
            self.assertFalse(r['data']['stopped_verified'])

    def test_cancelled_backend_and_corrupt_journal_do_not_prepare(self):
        with patch('agent.battery_backend.prepare_step') as prepare:
            self.b.cancel_and_wait(0)
            self.assertEqual(self.start()['status'],'rejected')
            other=self.make()
            other._path(self.request).write_text('{broken')
            self.assertEqual(other.invoke(self.op,self.payload,self.request)['status'],'unknown')
            prepare.assert_not_called()

    def test_uncertain_previous_request_blocks_new_motion(self):
        (self.b.directory/'previous.json').write_text('{"receipt":{"status":"unknown"}}')
        with patch('agent.battery_backend.prepare_step') as prepare:
            self.assertEqual(self.start()['status'],'rejected');prepare.assert_not_called()

    def test_d405_entry_preserves_completed_coarse_plan_before_clearing_mission_plan(self):
        import time
        from agent.tests.test_battery_execution_tools import BatteryExecutionToolsTests
        fixture=BatteryExecutionToolsTests();fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.assertTrue(fixture.run_coarse()['ok'])
        reviewed=copy.deepcopy(fixture.store.state['mission']['plan'])
        observed=fixture.observation('d405',time.time())
        with patch.object(fixture.tools,'_perception',return_value=observed):
            reply=fixture.tools.execute('observe_battery',{'camera':'d405'},'new_wrist')
        self.assertTrue(reply['ok'],reply)
        self.assertIsNone(fixture.store.state['mission']['plan'])
        self.assertEqual(fixture.store.state.get('coarse_observer_plan'),reviewed)

    def test_observer_search_is_journaled_and_never_replayed(self):
        self.store.state['pending']=None
        self.store.state['mission'].update(stage='COARSE_ALIGNED',
            plan=dict(self.payload['plan'],observer=dict(status='fixed_observer_candidate',
                target_wrist_base_mm=[251.767,-248.9933,700.5],rpy_deg=[-.3,-2.6,.1],
                column_path={'clear':True}),lift={'fixed':True}))
        self.store.state['coarse_observer_plan']=copy.deepcopy(self.store.state['mission']['plan'])
        self.store.state['mission']['plan']=None
        self.store.save()
        class SearchController(Controller):
            config={'arms':{'right':{'joint_limits_deg':[[-90,90],[-6,206],[-6,172],[-74,74],[-85,85],[-100,100]]}}}
            def snapshot(self):
                return dict(arms={'left':{'joints':[0.]*6,'fresh':True},
                                  'right':{'joints':[0.]*6,'fresh':True}},
                            body={'height':5.52,'fresh':True})
        class SearchStep:
            calls=[]
            def __init__(self,*args,**kwargs):self.calls.append('init')
            def start(self):self.calls.append('start');return {'status':'ready_to_observe','stop_required':False}
            def cancel(self):self.calls.append('cancel');return {'status':'cancelled'}
        settings=policy();coarse=dict(settings['coarse'],timing_limits=settings['timing_limits'])
        backend=BatteryControllerBackend(SearchController(),'owned',self.store,
            policies={'coarse':coarse},pick_setup=None,preflight=lambda *a:True,
            cancelled=lambda:False,timeout_s={'execute_coarse_alignment':1.,'execute_battery_approach':1.,'execute_pick':1.})
        with patch('agent.battery_backend.CoarseAlignmentStep',SearchStep):
            first=backend.adjust_observer_search('search',0,10.)
            second=backend.adjust_observer_search('search',0,10.)
        self.assertEqual(first,second)
        self.assertEqual(SearchStep.calls,['init','start'])

    def test_search_offsets_share_measured_xyz_rpy_after_backend_and_store_reopen(self):
        self.store.state['pending']=None
        self.store.state['mission'].update(stage='COARSE_ALIGNED',plan=None)
        reviewed=dict(self.payload['plan'],observer=dict(status='fixed_observer_candidate',
            target_wrist_base_mm=[300.,-100.,700.5],rpy_deg=[-.3,-2.6,.1],column_path={'clear':True}),
            lift={'fixed':True})
        self.store.state['coarse_observer_plan']=copy.deepcopy(reviewed)
        self.store.save()
        class SearchController(Controller):
            config={'arms':{'right':{'joint_limits_deg':[[-90,90],[-6,206],[-6,172],[-74,74],[-85,85],[-100,100]]}}}
            def __init__(self):self.q=np.zeros(6);self.lift=5.07686710357666
            def snapshot(self):
                return dict(arms={'left':{'joints':[0.]*6,'fresh':True},
                                  'right':{'joints':self.q.tolist(),'fresh':True}},
                            body={'height':self.lift,'fresh':True})
        controller=SearchController();model=ColumnModel()
        measured=model.wrist_transform(controller.q,controller.lift)
        class SearchStep:
            def __init__(self,controller,*args,**kwargs):
                self.controller=controller;self.plan=kwargs['observer_plan']
            def start(self):
                self.controller.q=np.asarray(self.plan['joint_waypoints_rad'][-1])
                return {'status':'ready_to_observe','stop_required':False}
            def cancel(self):return {'status':'cancelled'}
        settings=policy();coarse=dict(settings['coarse'],timing_limits=settings['timing_limits'])
        def backend(store):
            return BatteryControllerBackend(controller,'owned',store,
                policies={'coarse':coarse},pick_setup=None,preflight=lambda *a:True,
                cancelled=lambda:False,timeout_s={'execute_coarse_alignment':1.,'execute_battery_approach':1.,'execute_pick':1.})
        with patch('agent.battery_backend.CoarseAlignmentStep',SearchStep):
            first=backend(self.store).adjust_observer_search('search',0,10.)
            np.testing.assert_allclose(first['target_wrist_base_mm'],measured[:3,3]*1000+[0,0,10],atol=1e-6)
            origin=copy.deepcopy(self.store.state['observer_search_origin'])
            self.store.close();restored=RunStore.open(self.store.path);self.addCleanup(restored.close)
            second=backend(restored).adjust_observer_search('search',1,20.)
            np.testing.assert_allclose(second['target_wrist_base_mm'],measured[:3,3]*1000+[0,0,20],atol=1e-6)
            zero=backend(restored).adjust_observer_search('search',2,0.)
            np.testing.assert_allclose(zero['target_wrist_base_mm'],measured[:3,3]*1000,atol=1e-6)
        self.assertEqual(restored.state['observer_search_origin'],origin)
        np.testing.assert_allclose(Rotation.from_euler('xyz',origin['rpy_deg'],degrees=True).as_matrix(),
                                   measured[:3,:3],atol=1e-8)
        np.testing.assert_allclose(model.wrist_transform(controller.q,controller.lift),measured,atol=1e-6)
        self.assertEqual(plan_digest(restored.state['coarse_observer_plan']),plan_digest(reviewed))

if __name__=='__main__':unittest.main()
