import tempfile,threading,time,unittest
from pathlib import Path
from test_control import Controller,FakeAdapter,configured
from test_competition_task import Module
from test_t2_navigation_current_place import Navigation
from t2_test_task import make_t2_navigation_task
class TailLifecycle(unittest.TestCase):
 def test_place_receipt_then_wait_for_suffix_without_releasing_owner_or_replaying_place(self):
  config=configured();config['chassis_backend']='local';ctrl=Controller(config,FakeAdapter());ctrl.update_body(.18,[0.]*4,0.,0.,0.)
  for side in ('left','right'):ctrl.update_arm(side,[0.]*6,-1.,[0.]*6)
  nav=Navigation();tail_started=threading.Event();tail_done=threading.Event();trace=[]
  def complete(context):
   nav.trace.append('acknowledge_t2');tail_started.set()
   while not tail_done.wait(.01):
    if context['cancelled'].is_set():raise ValueError('cancelled')
   nav.active=False
  nav.complete_route=complete
  place=Module(trace,'place');place.readiness=lambda:dict(reference_valid=True)
  with tempfile.TemporaryDirectory() as d:
   task=make_t2_navigation_task(ctrl,Path(d),nav,place);self.addCleanup(task.close);self.addCleanup(tail_done.set)
   task.action('browser',dict(action='start',request_id='tail-test',expected_epoch=ctrl.command_epoch,environment_clear=True))
   self.assertTrue(nav.started.wait(1));self.assertEqual(trace,[]);nav.arrived.set();self.assertTrue(tail_started.wait(1))
   self.assertEqual(task.snapshot()['status'],'running');self.assertTrue(ctrl.snapshot()['competition_reserved']);self.assertFalse(ctrl.snapshot()['motion_reserved'])
   self.assertEqual([x[1] for x in trace if x[0]=='start'],['place']);tail_done.set()
   deadline=time.monotonic()+2
   while task.active['worker'].is_alive() and time.monotonic()<deadline:time.sleep(.01)
   self.assertEqual(task.snapshot()['status'],'completed');self.assertFalse(ctrl.snapshot()['competition_reserved']);self.assertEqual([x[1] for x in trace if x[0]=='start'],['place'])
 def test_page_describes_navigation_after_place(self):
  page=(Path(__file__).resolve().parents[1]/'public/t2-test.html').read_text()
  self.assertIn('剩余导航',page);self.assertIn('路线终点',page);self.assertNotIn('真实放置 → 结束',page)
if __name__=='__main__':unittest.main()
