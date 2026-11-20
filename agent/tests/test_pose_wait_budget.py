"""A live pose wait preserves the prepared execution identity and time budget."""
import tempfile,time,unittest,threading
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
from agent.pick_backend import ExtractionExecutionBackend
from agent.storage import RunStore
from agent.battery_flow import BatteryFlow

class WaitingStep:
 def __init__(self):self.polls=0;self.starts=0;self.timeouts=0;self.cancelled=False;self.waiting=threading.Event()
 def start(self):self.starts+=1;self.waiting.set();return dict(status='backing_chassis',waiting_for_pose=True)
 def poll(self):
  self.polls+=1
  return dict(status='chassis_stopped' if self.polls>=8 else 'backing_chassis',waiting_for_pose=self.polls<8)
 def timeout(self):self.timeouts+=1;return dict(status='timeout',stop_required=True)
 def cancel(self):self.cancelled=True;return dict(status='cancelled',stop_required=True)
class BudgetTests(unittest.TestCase):
 def backend(self,step,path):
  args={'plan_id':'p','plan':{'plan_id':'p','arm':'right'}}
  b=ExtractionExecutionBackend(step,request_id='same-extract',arguments=args,journal=Path(path)/'receipt.json',cancelled=lambda:False,timeout_s=.05)
  b.invoke('stow_for_carry',args,'same-extract');return b
 def test_pose_wait_longer_than_motion_budget_is_not_a_failed_extraction(self):
  with tempfile.TemporaryDirectory() as t:
   step=WaitingStep();b=self.backend(step,t);self.assertTrue(b.wait(2.));self.assertEqual(b.status('stow_for_carry','same-extract')['status'],'completed');self.assertEqual(step.starts,1);self.assertEqual(step.timeouts,0)
 def test_cancel_still_wins_during_pose_wait(self):
  with tempfile.TemporaryDirectory() as t:
   step=WaitingStep();b=self.backend(step,t);self.assertTrue(step.waiting.wait(1.));self.assertTrue(b.cancel_and_wait(1.));self.assertTrue(step.cancelled);self.assertNotEqual(b.status('stow_for_carry','same-extract')['status'],'completed')
 def test_flow_receipt_timeout_does_not_replay_or_abort_a_live_pose_wait(self):
  with tempfile.TemporaryDirectory() as t:
   store=RunStore.create(t);clock=[100.];calls=[]
   class Tools:
    def __init__(self):self.store=store
    def execute(self,name,args,ident):
     calls.append(name)
     if name=='stow_for_carry':store.state['pending']={'last_receipt':{'status':'running','data':{'extract_stage':'backing_chassis','waiting_for_pose':True}}}
     elif calls.count('get_execution_status')>=4:store.state['pending']=None;return dict(ok=True,data={'extracted':True})
     return dict(ok=False,data={},error={'code':'execution_running','message':'waiting'})
   flow=BatteryFlow(Tools(),review_plan=lambda *a:None,receipt_timeout_s=.2)
   fake=NS(time=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+1.))
   try:
    with patch('agent.battery_flow.time',fake):result=flow._call('stow_for_carry',{'plan_id':'p'})
    self.assertTrue(result['extracted']);self.assertEqual(calls.count('stow_for_carry'),1)
   finally:store.close()

class ContinuousRecoveryProof(unittest.TestCase):
 def test_repeated_invalid_pose_discards_previous_stop_window(self):
  from agent.pose_wait import PoseWait
  transport=NS(end_output=lambda:None,begin_task=lambda:None);w=PoseWait(transport);w.begin(True)
  for i in range(10):self.assertFalse(w.ready(([0,0,0],[0,0,0],10.+i*.1),10.+i*.1,allow_motion=True))
  w.begin(True)
  self.assertFalse(w.ready(([0,0,0],[0,0,0],11.),11.,allow_motion=True));self.assertTrue(w.ended)
if __name__=='__main__':unittest.main()
