"""Fixed sequence with inert tools: no ROS, SDK, camera or model calls."""
import copy
import tempfile
import unittest
from agent.storage import RunStore
from agent.battery_flow import BatteryFlow, plan_digest
from agent.config import Config

class Tools:
    def __init__(self,store):
        self.store=store;self.config=Config();self.calls=[];self.fail=None;self.corrections=0
        self.backends=self;self.cancelled_count=0
    def cancel_and_wait(self,timeout):self.cancelled_count+=1;return True
    def _save_pick_plan(self,plan):pass
    def available(self,name):return name=='verify_hold'
    def execute(self,name,args,ident):
        self.calls.append(name);m=self.store.state['mission'];data={}
        if self.fail==name:return dict(ok=False,error=dict(code='test_failure',message='test'),data=None)
        if name=='observe_battery':
            data=dict(status='localized',observation_id='head' if args['camera']=='d435' else 'wrist'+str(len(self.calls)))
        elif name=='preview_battery_grasp':data=dict(status='preview_complete',observation_id=args['observation_id'])
        elif name in ('plan_coarse_alignment','plan_battery_pick'):
            data=dict(plan_id='p'+str(len(self.calls)),arm='right',executable=False)
            if name=='plan_battery_pick':
                m['stage']='PICK_APPROACH_PLANNED' if not m.get('approach_plan') else 'PICK_PLANNED'
                if self.corrections and m.get('approach_plan'):
                    self.corrections-=1;m['stage']='PICK_CORRECTION_PLANNED'
            else:m['stage']='COARSE_PLANNED'
            m['plan']=copy.deepcopy(data)
        elif name=='execute_coarse_alignment':m['stage']='COARSE_ALIGNED'
        elif name=='execute_battery_approach':
            m['stage']='PICK_AT_PREGRASP';m['approach_plan']=m['plan'];m['plan']=None
        elif name=='execute_pick':m['stage']='PICK_EXECUTED'
        elif name=='verify_hold':m['stage']='HOLD_CONFIRMED';data={'hold_status':'confirmed'}
        elif name=='stow_for_carry':m['stage']='CARRY_READY';data={'carry_ready':True}
        return dict(ok=True,data=data,error=None)

def review(operation,plan):
    return dict(plan_sha256=plan_digest(plan),ik_checked=True,collision_checked=True,
                collision_free=True,environment_collision_checked=True)

class FlowTests(unittest.TestCase):
    def setUp(self):
        t=tempfile.TemporaryDirectory();self.addCleanup(t.cleanup)
        self.store=RunStore.create(t.name);self.addCleanup(self.store.close)
        self.tools=Tools(self.store)
    def flow(self,**kw):return BatteryFlow(self.tools,review_plan=review,**kw)
    def test_fixed_chain_with_reobserve_and_correction(self):
        self.tools.corrections=1
        out=self.flow().run()
        self.assertEqual(out['status'],'completed');self.assertTrue(out['grasp_confirmed'])
        names=self.tools.calls
        self.assertEqual(names.count('execute_battery_approach'),2)
        self.assertEqual(names.count('execute_pick'),1)
        self.assertEqual(names[-2:],["verify_hold","stow_for_carry"])
        self.assertEqual(self.store.state['mission']['stage'],'CARRY_READY')
        self.assertNotIn('capture_grasp_scene',names)
        self.assertEqual(names.count('preview_battery_grasp'),3)
    def test_missing_verifier_does_not_claim_success(self):
        self.tools.available=lambda name:False
        out=self.flow().run()
        self.assertEqual(out['status'],'awaiting_hold_verification');self.assertFalse(out['grasp_confirmed'])

    def test_extraction_failure_does_not_claim_the_flow_completed(self):
        self.tools.fail='stow_for_carry'
        result=self.flow().run()
        self.assertEqual(result['status'],'needs_attention')
        self.assertEqual(self.tools.calls[-2:],['verify_hold','stow_for_carry'])

    def test_correction_loop_is_not_stopped_by_the_initial_tool_call_budget(self):
        self.tools.corrections=25
        out=self.flow().run()
        self.assertEqual(out['status'],'completed')
        self.assertGreater(out['index'],self.tools.config.max_tool_calls)
        self.assertEqual(self.tools.calls.count('execute_battery_approach'),26)

    def test_correction_loop_still_honors_cancel(self):
        self.tools.corrections=25;base=self.tools.execute
        def execute(name,args,ident):
            result=base(name,args,ident)
            if name=='execute_battery_approach' and len(self.tools.calls)>15:
                (self.store.path/'cancel.flag').touch()
            return result
        self.tools.execute=execute
        self.assertEqual(self.flow().run()['status'],'cancelled')
        self.assertNotIn('execute_pick',self.tools.calls)
    def test_cancel_and_perception_failure_never_run_motion(self):
        self.tools.fail='observe_battery'
        out=self.flow().run();self.assertEqual(out['status'],'needs_attention')
        self.assertNotIn('execute_coarse_alignment',self.tools.calls)
        (self.store.path/'cancel.flag').touch()
        out=self.flow().run();self.assertFalse(out['grasp_confirmed'])
    def test_review_is_bound_to_exact_plan(self):
        flow=BatteryFlow(self.tools,review_plan=lambda op,p:dict(review(op,p),plan_sha256='wrong'))
        out=flow.run();self.assertEqual(out['status'],'needs_attention')
        self.assertNotIn('execute_coarse_alignment',self.tools.calls)
    def test_all_motion_stages_require_ik_evidence(self):
        def stage_review(operation,plan):
            return dict(review(operation,plan),ik_checked=False)
        out=BatteryFlow(self.tools,review_plan=stage_review).run()
        self.assertEqual(out['status'],'needs_attention')
        self.assertNotIn('execute_coarse_alignment',self.tools.calls)
    def test_finished_run_never_replays(self):
        self.flow().run();before=list(self.tools.calls)
        self.flow().run();self.assertEqual(before,self.tools.calls)

    def test_pending_receipt_resolves_before_next_phase(self):
        flow=self.flow();base=self.tools.execute
        def execute(name,args,ident):
            if name=='execute_coarse_alignment':
                self.store.state['pending']={'last_receipt':{'data':{}}}
                return dict(ok=False,error=dict(code='execution_pending',message='waiting'))
            if name=='get_execution_status':
                self.store.state['pending']=None
                return base('execute_coarse_alignment',args,ident)
            return base(name,args,ident)
        self.tools.execute=execute
        self.assertEqual(flow.run()['status'],'completed')
        self.assertEqual(flow.flow['polls'],1)

    def test_unknown_receipt_cancels_and_never_proceeds(self):
        flow=self.flow();base=self.tools.execute
        def execute(name,args,ident):
            if name in ('execute_coarse_alignment','get_execution_status'):
                self.store.state['pending']={'last_receipt':{'data':{'recovery_required':True}}}
                return dict(ok=False,error=dict(code='execution_unknown',message='uncertain'))
            return base(name,args,ident)
        self.tools.execute=execute
        self.assertEqual(flow.run()['status'],'needs_attention')
        self.assertEqual(self.tools.cancelled_count,1)
        self.assertNotIn('execute_battery_approach',self.tools.calls)

    def test_persisted_command_keeps_original_arguments_after_completion(self):
        flow=self.flow();flow.flow.update(phase='approach_execute',index=5,
            command={'name':'execute_battery_approach','arguments':{'plan_id':'original'}})
        self.store.state['mission'].update(stage='PICK_AT_PREGRASP',plan=None)
        seen=[]
        def execute(name,args,ident):
            seen.append((name,args,ident))
            if name=='execute_battery_approach':return dict(ok=True,data={'pose_reached':True})
            return dict(ok=True,data={'status':'target_not_found'})
        self.tools.execute=execute
        out=flow.run()
        self.assertEqual(out['status'],'needs_attention')
        self.assertEqual(seen[0],('execute_battery_approach',{'plan_id':'original'},'fixed_5'))
        self.assertEqual(seen[1][0],'observe_battery')

    def test_cancel_during_verifier_never_reports_success(self):
        flow=self.flow();base=self.tools.execute
        def execute(name,args,ident):
            result=base(name,args,ident)
            if name=='verify_hold':(self.store.path/'cancel.flag').touch()
            return result
        self.tools.execute=execute
        out=flow.run()
        self.assertEqual(out['status'],'cancelled')
        self.assertFalse(out['grasp_confirmed'])

    def test_second_runner_does_not_dispatch(self):
        flow=self.flow();self.store.battery_flow_lock.acquire()
        try:self.assertEqual(flow.run()['reason'],'Flow already running')
        finally:self.store.battery_flow_lock.release()
        self.assertEqual(self.tools.calls,[])

if __name__=='__main__':unittest.main()
