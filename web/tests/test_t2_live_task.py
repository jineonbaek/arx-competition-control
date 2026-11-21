"""Real task scheduler with inert modules/controller, never hardware."""
import unittest,tempfile,time
from pathlib import Path
from test_control import Controller,FakeAdapter,configured,Rejected
from test_competition_task import Module
class T2LiveTaskTests(unittest.TestCase):
 def test_real_module_sequence_without_t1_presets_or_lift_and_one_owner(self):
  from t2_direct_task import T2CurrentPositionTask
  trace=[];adapter=FakeAdapter();ctrl=Controller(configured(),adapter);ctrl.update_body(.18,[0.]*4,0.,0.,0.)
  for side in ('left','right'):ctrl.update_arm(side,[0.]*6,-1.,[0.]*6)
  modules={key:Module(trace,key) for key in ('navigate_t2','place','navigate_exit')};modules['navigate_t2'].navigation=True;modules['navigate_exit'].navigation=True
  with tempfile.TemporaryDirectory() as d:
   task=T2CurrentPositionTask(ctrl,Path(d),modules=modules);self.addCleanup(task.close)
   payload=dict(action='start',request_id='one',expected_epoch=ctrl.command_epoch,environment_clear=True)
   task.action('browser',payload);deadline=time.monotonic()+2
   while task.active['worker'].is_alive() and time.monotonic()<deadline:time.sleep(.01)
   self.assertFalse(task.active['worker'].is_alive());self.assertEqual(task.snapshot()['status'],'completed')
   starts=[x for x in trace if x[0]=='start'];self.assertEqual([x[1] for x in starts],['place'])
   self.assertIsNotNone(starts[0][3]);self.assertEqual(adapter.calls,[])
   task.action('browser',payload);self.assertEqual(len([x for x in trace if x[0]=='start']),1)
   self.assertFalse(ctrl.snapshot()['competition_reserved']);self.assertFalse(ctrl.snapshot()['motion_reserved'])
if __name__=='__main__':unittest.main()
