import tempfile
import unittest
from pathlib import Path
from agent.storage import RunStore
from agent import placement_flow as f

class Backend:
    def __init__(self):self.calls=[];self.cancel=False;self.failure=None
    def call(self,name,*args):
        self.calls.append(name)
        if self.failure==name:raise ValueError('blocked')
        return dict(status='confirmed',name=name)
    def stop(self):self.calls.append('stop')

class PlacementFlowTests(unittest.TestCase):
    def make(self):
        t=tempfile.TemporaryDirectory();self.addCleanup(t.cleanup);s=RunStore.create(Path(t.name));self.addCleanup(s.close);return s,Backend()
    def test_order_and_observation_before_release(self):
        self.assertTrue(hasattr(f,'PlacementFlow'),'fixed placement flow is required')
        s,b=self.make();r=f.PlacementFlow(s,b).run()
        self.assertEqual(r['status'],'completed');self.assertTrue(r['placement_confirmed'])
        self.assertEqual(b.calls,['held','head','preplace','wrist','correct','reobserve','descend','release','withdraw','verify'])
    def test_each_failed_stage_cannot_release_early(self):
        for stage in ['held','head','preplace','wrist','correct','reobserve','descend']:
            s,b=self.make();b.failure=stage;r=f.PlacementFlow(s,b).run();self.assertNotEqual(r['status'],'completed');self.assertNotIn('release',b.calls);self.assertEqual(b.calls[-1],'stop')
    def test_cancel_preserves_hold_and_stops(self):
        s,b=self.make();(s.path/'cancel.flag').touch();r=f.PlacementFlow(s,b).run();self.assertEqual(r['status'],'cancelled');self.assertNotIn('release',b.calls)
    def test_ambiguous_restart_never_replays(self):
        s,b=self.make();s.state['placement_flow']={'status':'running','phase':'release','inflight':True};s.save()
        r=f.PlacementFlow(s,b).run();self.assertEqual(r['status'],'recovery_required');self.assertNotIn('release',b.calls)
    def test_false_receipt_does_not_complete(self):
        s,b=self.make();b.call=lambda *args:dict(status='unknown')
        r=f.PlacementFlow(s,b).run();self.assertNotEqual(r['status'],'completed');self.assertFalse(r['placement_confirmed'])
    def test_failed_post_release_verification_is_not_success(self):
        s,b=self.make();b.failure='verify';r=f.PlacementFlow(s,b).run();self.assertFalse(r['placement_confirmed']);self.assertIn('release',b.calls)

if __name__=='__main__':unittest.main()
