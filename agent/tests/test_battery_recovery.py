"""Bounded software recovery; mocked perception, no cameras or actuators."""
import copy
import unittest
from unittest.mock import patch
from agent.contracts import AgentError
from agent.storage import atomic_json
from agent.tests import test_battery_pick_plan as fixtures

class BatteryRecoveryTests(unittest.TestCase):
    def setUp(self):
        fixtures.BatteryPickPlanTests.setUp(self)
        self.store.state['coarse_alignment']='fixture'
    def head(self):
        self.store.state['mission']['stage']='IDLE'
        return self.tools.execute('observe_battery',{'camera':'d435'},'head')
    def test_transient_head_failure_recovers_inside_one_tool_call(self):
        with patch('agent.grasp_tools.observe',side_effect=[
            dict(status='target_not_found',observation_id='a'),
            dict(status='depth_rejected',observation_id='b'),
            dict(status='localized',observation_id='c')]) as observe:
            r=self.head()
        self.assertTrue(r['ok'],r)
        self.assertEqual(r['data']['status'],'localized')
        self.assertEqual(observe.call_count,3)
        self.assertEqual(len({c.args[2] for c in observe.call_args_list}),3)
        self.assertEqual(observe.call_args_list[1].args[2],'head/retry_1')
        self.assertEqual(observe.call_args_list[2].args[2],'head/retry_2')
        self.assertEqual(self.store.state['mission']['stage'],'HEAD_LOCALIZED')
        self.assertEqual(len(r['data']['recovery']['attempts']),3)
        self.assertFalse(r['data']['recovery']['exhausted'])
    def test_persistent_failure_is_bounded_and_cannot_reset_budget_with_new_call(self):
        with patch('agent.grasp_tools.observe',return_value=dict(status='recognition_rejected',observation_id='a')) as observe:
            r=self.head()
            again=self.tools.execute('observe_battery',{'camera':'d435'},'again')
        self.assertEqual(observe.call_count,3)
        self.assertTrue(r['data']['recovery']['exhausted'])
        self.assertEqual(again['error']['code'],'observation_budget')
        self.assertEqual(self.store.state['mission']['stage'],'IDLE')
    def test_fault_and_cancel_are_not_automatically_retried(self):
        with patch('agent.grasp_tools.observe',side_effect=AgentError('invalid_run','bad evidence')) as observe:
            r=self.head()
        self.assertEqual(r['error']['code'],'invalid_run');self.assertEqual(observe.call_count,1)
        def cancel(*args):
            (self.store.path/'cancel.flag').touch()
            return dict(status='target_not_found',observation_id='a')
        with patch('agent.grasp_tools.observe',side_effect=cancel) as observe:
            r=self.tools.execute('observe_battery',{'camera':'d435'},'cancel')
        self.assertEqual(r['error']['code'],'cancelled');self.assertEqual(observe.call_count,1)
    def test_pose_rejection_reacquires_then_plans_with_new_evidence_and_no_tcp(self):
        self.store.state['battery_observation_attempts']={self.tools._observation_budget_key('d405'):1}
        new_id=self.ident
        def observe(registry,camera,call):
            nonlocal new_id
            import io,time,hashlib
            import numpy as np
            from agent.grasp_scene import save_frame,CALIBRATION
            f=copy.deepcopy(self.frame);f['id']=np.array('recovery_new')
            for key in ('image_time','arm_time','body_time'):f[key]=np.array(time.time())
            buf=io.BytesIO();np.savez(buf,**f)
            new_id=save_frame(self.store,buf.getvalue(),source='synthetic',calibration_sha256=hashlib.sha256(CALIBRATION.read_bytes()).hexdigest())['observation_id']
            path=self.store.path/(new_id+'.json')
            atomic_json(path,dict(status='localized',camera=camera,observation_id=new_id))
            self.store.state['battery_observations'][new_id]=path.name
            self.store.state['latest_battery_observations'][camera]=new_id
            return dict(status='localized',observation_id=new_id)
        reports=[]
        def pipeline(store,ident,clearance,output,*args,**kwargs):
            report=copy.deepcopy(self.report);report['observation_id']=ident
            report['camera_handoff']['wrist_observation_id']=ident
            report['geometry']['snapshot_id']='recovery_new'
            if not reports:report['status']='pose_rejected'
            reports.append(ident);output.mkdir(parents=True,exist_ok=True)
            atomic_json(output/'result.json',report)
            return report
        with patch('agent.grasp_tools.observe',side_effect=observe),patch('agent.grasp_tools.pipeline',side_effect=pipeline):
            r=self.tools.execute('preview_battery_grasp',dict(observation_id=self.ident,clearance_mm=10),'preview')
        self.assertTrue(r['ok'],r);self.assertEqual(r['data']['status'],'preview_complete')
        self.assertEqual(reports,[self.ident,new_id]);self.assertNotEqual(self.ident,new_id)
        p=self.tools.execute('plan_battery_pick',{'observation_id':new_id},'plan')
        self.assertTrue(p['ok'],p);self.assertEqual(p['data']['status'],'tcp_pending')
        self.assertIsNone(self.store.state['pending'])

if __name__=='__main__':unittest.main()
