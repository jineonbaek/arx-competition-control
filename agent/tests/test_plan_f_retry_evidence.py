"""Offline coverage of Plan F retry and post-approach observation timing."""
import json
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from agent.battery_tools import BatteryExecutionTools
from agent.config import Config
from agent.storage import RunStore


class PlanFRetryEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.store=RunStore.create(self.temp.name);self.addCleanup(self.store.close)
        self.store.state['battery_task_settings']={'vision_mode':'plan_f'}
        self.store.state['mission'].update(stage='COARSE_ALIGNED',plan=None)
        self.tools=BatteryExecutionTools(Config(),self.store,observations=object(),backends=SimpleNamespace())

    def observation(self,captured):
        obs=self.store.save_observation({'cameras':{'d405':{'captured_at':captured}}},{})
        ident=obs['observation_id']
        result={'observation_id':ident,'camera':'d405','status':'localized'}
        path=self.store.path/(ident+'.json');path.write_text(json.dumps(result))
        self.store.state.setdefault('battery_observations',{})[ident]=path.name
        self.store.state.setdefault('latest_battery_observations',{})['d405']=ident
        return result

    def test_initial_pose_rejection_recaptures_and_retries_without_observer_move(self):
        old=self.observation(time.time());seen=[];captured=[]
        def observe(*args):
            result=self.observation(time.time());captured.append(result);return result
        def pipeline(store,ident,clearance,output,*args,**kwargs):
            seen.append(ident);output.mkdir(parents=True,exist_ok=True)
            result={'observation_id':ident,'status':'pose_rejected' if len(seen)==1 else 'preview_complete',
                    'message':'fixture'}
            (output/'result.json').write_text(json.dumps(result));return result
        with patch('agent.grasp_tools.observe',side_effect=observe),patch('agent.grasp_tools.pipeline',side_effect=pipeline):
            result=self.tools.execute('preview_battery_grasp',
                {'observation_id':old['observation_id'],'clearance_mm':50},'preview')
        self.assertTrue(result['ok'],result)
        self.assertEqual(result['data']['status'],'preview_complete')
        self.assertEqual(seen,[old['observation_id'],captured[0]['observation_id']])
        self.assertEqual(result['data']['observation_id'],captured[0]['observation_id'])
        self.assertNotIn('evidence_barriers',self.store.state['mission'])

    def test_post_approach_old_frame_still_rejected_and_fresh_frame_accepted(self):
        before=time.time();old=self.observation(before)
        self.store.state['mission'].update(stage='PICK_AT_PREGRASP',approach_plan={'plan_id':'approach'},
            evidence_barriers={'PICK_AT_PREGRASP':{'completed_at':time.time(),'previous_observation_ids':[old['observation_id']]}})
        with patch('agent.grasp_tools.observe',return_value=old):
            result=self.tools.execute('observe_battery',{'camera':'d405'},'old')
        self.assertEqual(result['error']['code'],'post_action_evidence_required')
        with patch('agent.grasp_tools.observe',side_effect=lambda *a:self.observation(time.time())):
            result=self.tools.execute('observe_battery',{'camera':'d405'},'fresh')
        self.assertTrue(result['ok'],result)

    def test_plan_a_still_requires_observer_completion(self):
        self.tools.vision_mode='d405';old=self.observation(time.time())
        self.store.state['coarse_alignment']='coarse.json'
        with patch.object(self.tools,'_localized',return_value='head'),patch('agent.grasp_tools.observe',return_value=old):
            result=self.tools.execute('observe_battery',{'camera':'d405'},'old')
        self.assertEqual(result['error']['code'],'post_action_evidence_required')


if __name__=='__main__':
    unittest.main()
