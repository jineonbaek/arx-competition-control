"""Real dispatch/backend journals with inert Controller steps, never hardware."""
import copy
import tempfile
import time
import unittest
from unittest.mock import patch
from agent.config import Config
from agent.storage import RunStore,atomic_json
from agent.battery_workflow import build_workflow
from agent.tests.test_battery_flow import review
from agent.tests.test_battery_backend import Controller
from agent.tests.test_coarse_backend import Step

class WorkflowTests(unittest.TestCase):
    def setUp(self):
        t=tempfile.TemporaryDirectory();self.addCleanup(t.cleanup)
        self.store=RunStore.create(t.name);self.addCleanup(self.store.close)
        self.config=Config();self.config.motion_enabled=True
        self.controller=Controller();self.gripper=-3.
        self.controller.snapshot=lambda:dict(arms={'right':dict(fresh=True,gripper=self.gripper)})
    def build(self,**kw):
        flow=build_workflow(self.config,self.store,self.controller,'owned',
            policies={'pick':{'hold_gripper_range':[-1.5,-.5]}},
            refinement_policy={},timing_limits={},alignment_tolerance_deg=3,j6_tolerance_deg=5,
            preflight=lambda *a:True,review_plan=review,observations=object(),
            timeout_s={k:1. for k in ('execute_coarse_alignment','execute_battery_approach','execute_pick')},**kw)
        self.addCleanup(lambda:flow.tools.backends.cancel_and_wait(2))
        return flow
    def observation(self,camera):
        return self.store.save_observation({'captured_at':time.time(),
            'cameras':{camera:{'captured_at':time.time(),'image_size_px':[640,480]}}},{})['observation_id']
    def coarse(self,flow):
        obs=self.observation('d435')
        path=self.store.path/'head.json'
        atomic_json(path,dict(status='localized',camera='d435',observation_id=obs))
        self.store.state.update(latest_battery_observations={'d435':obs},battery_observations={obs:path.name},coarse_alignment='fixture')
        plan=dict(plan_id='p',arm='right',observation_id=obs,
            observer={'status':'fixed_observer_candidate','joints_rad':[0.]*6,'column_path':{'clear':True}},
            lift={'fixed':True,'target_rad':5.52,'acceptance_rad':5.0,'current_rad':5.0})
        self.store.state['mission'].update(stage='COARSE_PLANNED',plan=plan)
        flow.flow.update(phase='coarse_execute',index=2)
        self.store.save()
    def test_assembly_uses_measured_tcp_and_has_no_effects(self):
        with patch('agent.battery_backend.prepare_step') as prepare:
            flow=self.build()
            self.assertAlmostEqual(flow.tools.pick_setup['T_wrist_from_tool'][0][3],.14814204045)
            self.assertEqual(list(flow.tools.backends.directory.iterdir()),[])
            prepare.assert_not_called()
    def test_real_dispatch_completes_coarse_once_then_requires_new_observation(self):
        flow=self.build();self.coarse(flow);step=Step();step.finish.set()
        with patch('agent.battery_backend.prepare_step',return_value=step) as prepare, patch(
                'agent.grasp_tools.observe',return_value={'status':'target_not_found'}):
            out=flow.run()
            self.assertEqual(out['status'],'needs_attention',out)
            self.assertEqual(self.store.state['mission']['stage'],'COARSE_ALIGNED')
            self.assertIn('COARSE_ALIGNED',self.store.state['mission']['evidence_barriers'])
            self.assertEqual(step.calls,['start']);prepare.assert_called_once()
            self.assertIsNone(self.store.state['pending'])
            flow.run();prepare.assert_called_once()
    def test_motion_disabled_never_prepares_step(self):
        flow=self.build();self.coarse(flow);self.config.motion_enabled=False
        with patch('agent.battery_backend.prepare_step') as prepare:
            out=flow.run();self.assertEqual(out['status'],'needs_attention');prepare.assert_not_called()
    def test_post_close_verification_uses_gripper_feedback_range(self):
        flow=self.build()
        m=self.store.state['mission'];m.update(stage='PICK_EXECUTED',target={'target_id':'battery'},plan={})
        result=flow.tools.execute('verify_hold',{},'outside')
        self.assertFalse(result['ok']);self.assertEqual(m['stage'],'PICK_EXECUTED')
        self.gripper=-1.
        result=flow.tools.execute('verify_hold',{},'inside')
        self.assertTrue(result['ok'],result);self.assertEqual(m['stage'],'HOLD_CONFIRMED')
        self.assertEqual(result['data']['gripper_feedback'],-1.)

if __name__=='__main__':unittest.main()
