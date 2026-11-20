"""Exercise real escape/special/retreat adapters; inert observations only."""
import io,json,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import escape_mode,special_runtime
from chassis_command import WebChassis

def state():
 return dict(execution_enabled=True,body=dict(fresh=True),chassis_active=False,
             motion_reserved=True,nav_sdk_ready=True,capabilities=dict(chassis=''),
             command_epoch=9,chassis_scheduler=dict(output_seq=101,maintenance=False))

class IntegrationTests(unittest.TestCase):
 def transport(self,s):
  t=WebChassis(submit=lambda m:dict(ok=True));t.native_sdk=True
  t.client=NS(open=lambda *a,**k:io.BytesIO(json.dumps(s).encode()))
  return t
 def test_escape_console_does_not_depend_on_removed_sequence_or_arm_epoch(self):
  s=state();e=escape_mode.EscapeMode.__new__(escape_mode.EscapeMode)
  e.d=NS(transport=self.transport(s),active=False);e.control=None
  e.console(begin=True)
  s['command_epoch']+=10;s['chassis_scheduler']['output_seq']+=50
  e.console()
 def test_replan_readiness_does_not_use_arm_epoch_or_own_output_counter(self):
  from mppi_goal_drive import MppiDriver
  s=state();s['motion_reserved']=False;n=NS(transport=self.transport(s))
  before=MppiDriver.recovery_console(n)
  s['chassis_scheduler']['output_seq']+=8;s['command_epoch']+=30
  self.assertEqual(MppiDriver.recovery_console(n),before)
 def test_special_mode_does_not_treat_arm_or_ordinary_writer_as_exclusive(self):
  s=state();t=self.transport(s);source=[100.]
  def poses():
   source[0]+=.05
   return np.zeros(3),np.zeros(3),source[0]
  n=NS(seen=1,current_intent=lambda _:None,transport=t,active=False,
       waypoints=NS(blocking=lambda:False,route=NS(direct_source=dict(pose=[0,0,0]),
         direct_points=lambda:[],advance_direct=lambda _:None),show=lambda *a:None),
       escape=NS(clear=lambda:None),special_poses=poses,status=lambda _:None,
       spin=lambda _:None,now=lambda:100.,brake=lambda:None,end_output=lambda:None)
  chain=NS(target=np.zeros(2),step=lambda *a:NS(passed_ids=[],command=(0.,0.,0.),terminal_near=True))
  with patch.object(special_runtime,'cancel_mppi'),patch.object(special_runtime,'StraightChain',return_value=chain),patch.object(special_runtime,'StopWindow',return_value=NS(add=lambda *a:True)):
   self.assertEqual(special_runtime.run_special(n),'arrived_stopped')
 def test_successful_work_retreat_closes_output_identity(self):
  from agent.chassis_retreat import ChassisRetreatStep
  sent=[];t=WebChassis(submit=lambda m:sent.append(m) or dict(ok=True))
  step=ChassisRetreatStep(None,None);step.transport=t;step.reader=NS(close=lambda:None)
  t.send((0.,0.,0.));step.close()
  self.assertEqual(sent[-1]['type'],'chassis.end_output')
  with self.assertRaises(RuntimeError):t.send((.1,0.,0.))

if __name__=='__main__':unittest.main()
