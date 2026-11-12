"""Opt-in Agent stage routing with synthetic observations and an inert motion step."""
import copy
import json
import tempfile
import time
import unittest
from unittest.mock import patch
from agent.config import Config
from agent.storage import RunStore
from agent.battery_tools import BatteryExecutionTools, EXECUTION_TOOLS
from agent.pick_backend import CoarseExecutionBackend
from agent.tests.test_coarse_backend import Step


class BatteryExecutionToolsTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.store=RunStore.create(self.temp.name);self.addCleanup(self.store.close)
        self.head=self.observation('d435',time.time())
        self.plan=dict(plan_id='coarse',arm='right',observation_id=self.head['observation_id'],
            executable=True,ik_checked=True,collision_checked=True,collision_free=True,
            environment_collision_checked=True,
            observer={'status':'fixed_observer_candidate','joints_rad':[0.]*6,'column_path':{'clear':True}},
            lift={'fixed':True,'target_rad':5.52,'current_rad':5.0})
        self.store.state.update(coarse_alignment='coarse.json')
        self.store.state['mission'].update(stage='COARSE_PLANNED',plan=copy.deepcopy(self.plan))
        self.payload=dict(plan_id='coarse',task=None,target=None,slot=None,plan=copy.deepcopy(self.plan))
        self.step=Step();self.ident=self.store.state['run_id']+':move'
        self.backend=CoarseExecutionBackend(self.step,request_id=self.ident,arguments=self.payload,
            journal=self.store.path/'receipt.json',cancelled=lambda:self.store.cancelled,timeout_s=2.)
        self.addCleanup(lambda:self.backend.cancel_and_wait(2))
        self.tools=BatteryExecutionTools(Config(motion_enabled=True),self.store,
            observations=object(),backends=self.backend,allowed_tools=EXECUTION_TOOLS)
    def observation(self,camera,captured):
        obs=self.store.save_observation(dict(cameras={camera:{'captured_at':captured}}),{})
        ident=obs['observation_id'];e=dict(observation_id=ident,camera=camera,status='localized')
        p=self.store.path/(ident+'.json');p.write_text(json.dumps(e))
        self.store.state.setdefault('battery_observations',{})[ident]=p.name
        self.store.state.setdefault('latest_battery_observations',{})[camera]=ident
        return e
    def run_coarse(self):
        r=self.tools.execute('execute_coarse_alignment',{'plan_id':'coarse'},'move')
        self.assertFalse(r['ok']);self.assertTrue(self.step.seen.wait(1))
        self.step.finish.set();self.assertTrue(self.backend.wait(1))
        return self.tools.execute('get_execution_status',{},'query')
    def test_agent_executes_coarse_then_requires_new_camera_frame(self):
        old=self.observation('d405',time.time())
        r=self.tools.execute('observe_battery',{'camera':'d405'},'early')
        self.assertEqual(r['error']['code'],'stage_not_allowed')
        r=self.run_coarse();self.assertTrue(r['ok'],r)
        self.assertEqual(self.store.state['mission']['stage'],'COARSE_ALIGNED')
        with patch('agent.grasp_tools.observe',return_value=old):
            r=self.tools.execute('observe_battery',{'camera':'d405'},'old')
        self.assertEqual(r['error']['code'],'post_action_evidence_required')
        self.assertNotIn('d405',self.store.state['latest_battery_observations'])
        def fresh(*args):return self.observation('d405',time.time())
        with patch('agent.grasp_tools.observe',side_effect=fresh):
            r=self.tools.execute('observe_battery',{'camera':'d405'},'fresh')
        self.assertTrue(r['ok'],r)
        self.tools.execute('execute_coarse_alignment',{'plan_id':'coarse'},'move')
        self.assertEqual(self.step.calls,['start'])
        r=self.tools.execute('execute_coarse_alignment',{'plan_id':'coarse'},'again')
        self.assertFalse(r['ok']);self.assertEqual(self.step.calls,['start'])
    def test_nonexecutable_proposal_and_wrong_plan_cannot_move(self):
        for key in ('executable','collision_checked','collision_free','environment_collision_checked'):
            self.store.state['mission']['plan']=dict(self.plan,**{key:False})
            r=self.tools.execute('execute_coarse_alignment',{'plan_id':'coarse'},key)
            self.assertEqual(r['error']['code'],'plan_not_verified',r)
        self.store.state['mission']['plan']=dict(self.plan,observer={
            'status':'fixed_observer_unavailable','joints_rad':None})
        r=self.tools.execute('execute_coarse_alignment',{'plan_id':'coarse'},'observer')
        self.assertEqual(r['error']['code'],'plan_not_verified',r)
        r=self.tools.execute('execute_coarse_alignment',{'plan_id':'other'},'wrong')
        self.assertEqual(r['error']['code'],'plan_missing')
        self.assertEqual(self.step.calls,[]);self.assertIsNone(self.store.state['pending'])
    def test_disabled_motion_and_pending_call_block_dependent_operations(self):
        self.tools.config.motion_enabled=False
        self.assertEqual(self.tools.execute('execute_coarse_alignment',{'plan_id':'coarse'},'off')['error']['code'],'motion_disabled')
        self.tools.config.motion_enabled=True
        self.tools.execute('execute_coarse_alignment',{'plan_id':'coarse'},'move')
        self.assertTrue(self.step.seen.wait(1))
        r=self.tools.execute('observe_battery',{'camera':'d405'},'during')
        self.assertEqual(r['error']['code'],'execution_pending')
        self.backend.cancel_and_wait(1)
        r=self.tools.execute('get_execution_status',{},'cancelled')
        self.assertFalse(r['ok']);self.assertIsNotNone(self.store.state['pending'])
        self.assertEqual(self.store.state['mission']['stage'],'COARSE_PLANNED')

    def test_execution_pending_heartbeats_are_not_written_to_event_log(self):
        self.tools.execute('execute_coarse_alignment',{'plan_id':'coarse'},'move')
        self.assertTrue(self.step.seen.wait(1))
        result=self.tools.execute('get_execution_status',{},'heartbeat')
        self.assertEqual(result['error']['code'],'execution_pending')
        events=[json.loads(line) for line in (self.store.path/'events.jsonl').read_text().splitlines()]
        self.assertFalse(any(row['event']=='tool_result'
            and (row.get('data') or {}).get('error',{}).get('code')=='execution_pending'
            for row in events))
        self.step.finish.set();self.assertTrue(self.backend.wait(1))
    def test_current_planner_proposal_is_stored_without_motion_permission(self):
        self.store.state['mission'].update(stage='HEAD_LOCALIZED',plan=None)
        proposal=dict(observation_id=self.head['observation_id'],executable=False,tcp=None)
        with patch('agent.grasp_tools.plan_coarse',return_value=proposal):
            r=self.tools.execute('plan_coarse_alignment',{'observation_id':self.head['observation_id']},'plan')
        self.assertTrue(r['ok'],r);self.assertTrue(r['data']['plan_id'].startswith('coarse_'))
        self.assertFalse(self.store.state['mission']['plan']['executable'])
        self.assertEqual(self.step.calls,[])
    def test_head_observation_sets_stage_and_failed_retry_invalidates_plan(self):
        self.store.state['mission'].update(stage='IDLE',plan=None)
        with patch('agent.grasp_tools.observe',return_value=self.head):
            r=self.tools.execute('observe_battery',{'camera':'d435'},'head')
        self.assertTrue(r['ok'],r)
        self.assertEqual(self.store.state['mission']['stage'],'HEAD_LOCALIZED')
        with patch('agent.grasp_tools.observe',side_effect=ValueError('camera offline')):
            r=self.tools.execute('observe_battery',{'camera':'d435'},'retry')
        self.assertFalse(r['ok']);self.assertEqual(self.store.state['mission']['stage'],'IDLE')
        self.assertIsNone(self.store.state['mission']['plan'])
        self.assertEqual(self.step.calls,[])

    def test_new_archive_with_pre_motion_camera_timestamp_is_rejected(self):
        before=time.time();self.assertTrue(self.run_coarse()['ok'])
        def stale(*args):return self.observation('d405',before)
        with patch('agent.grasp_tools.observe',side_effect=stale):
            r=self.tools.execute('observe_battery',{'camera':'d405'},'stale')
        self.assertEqual(r['error']['code'],'post_action_evidence_required')
        self.assertNotIn('d405',self.store.state['latest_battery_observations'])
        self.assertEqual(self.step.calls,['start'])

    def test_d405_target_not_found_searches_z_then_retries_only_on_new_frame(self):
        self.assertTrue(self.run_coarse()['ok'])
        calls=[]
        def observe_search(*args):
            calls.append(args)
            if len(calls)==1:
                return {'observation_id':'missing','camera':'d405','status':'target_not_found','message':''}
            observed=self.observation('d405',time.time())
            relative=observed['observation_id']+'.json'
            path=self.store.path/relative
            path.write_text(json.dumps(dict(observation_id=observed['observation_id'],camera='d405',status='localized')))
            self.store.state['battery_observations'][observed['observation_id']]=relative
            self.store.state['latest_battery_observations']['d405']=observed['observation_id']
            self.store.save()
            return dict(observed,location={'surface_base_mm':[400,0,700]})
        moves=[]
        self.tools.backends.observer_search_offsets=lambda:[10.0,20.0]
        self.tools.backends.adjust_observer_search=lambda call_id,index,offset:moves.append((call_id,index,offset)) or {'status':'completed'}
        with patch('agent.grasp_tools.observe',side_effect=observe_search):
            r=self.tools.execute('observe_battery',{'camera':'d405'},'search')
        self.assertTrue(r['ok'],r)
        self.assertEqual(moves,[('search',0,10.0)])
        self.assertEqual(r['data']['status'],'localized')

    def test_bad_completion_does_not_advance_mission(self):
        from agent.mission import Mission
        from agent.contracts import AgentError
        for data in ({},dict(observer_ready=True,new_observation_required=True,achieved_lift_rad=float('nan'))):
            with self.assertRaises(AgentError):
                Mission(self.store.state['mission']).apply('execute_coarse_alignment',data,self.payload)
            self.assertEqual(self.store.state['mission']['stage'],'COARSE_PLANNED')

if __name__=='__main__':unittest.main()
