"""Same-entry/same-chain recovery in the intentionally map-independent mode."""
import time,unittest
import numpy as np,rclpy
from std_msgs.msg import String
import test_special_ros as f

class SpecialRetention(unittest.TestCase):
    setUpClass=classmethod(lambda cls:rclpy.init())
    tearDownClass=classmethod(lambda cls:rclpy.shutdown())
    setUp=f.SpecialROS.setUp;tearDown=f.SpecialROS.tearDown
    refresh=f.SpecialROS.refresh;add=f.SpecialROS.add;start_special=f.SpecialROS.start_special
    def prepare(self,kind):
        self.start_special(.23);self.simulate=True;self.identity=self.n.transport.current
        self.n.destroy_timer(self.timer);began=time.monotonic();self.drifted=False;self.paused_at=None
        status=self.n.status
        def log(v):
            status(v)
            if v.startswith('paused_navigation'):self.paused_at=time.monotonic()
            if kind=='drift' and v=='special_verifying_stopped' and not self.drifted:
                self.position[0]-=.015;self.drifted=True
        self.n.status=log
        def feed():
            now=time.monotonic()
            if now-began>10:raise AssertionError('special fixture timeout')
            self.refresh()
            if kind=='gap' and .2<now-began<.75:self.n.sensor={}
            if self.paused_at and now-self.paused_at>.2:self.n.cancel('cancelled')
        self.timer=self.n.create_timer(.02,feed);feed()
    def test_scan_gap_resumes_same_straight_chain_and_stops(self):
        self.prepare('gap');self.assertEqual(self.n.run_goal(),'arrived_stopped')
        self.assertEqual(self.n.transport.current,self.identity);self.assertIsNone(self.paused_at)
        self.assertLessEqual(abs(self.position[0]-.23),.10)
    def test_long_parked_gap_does_not_exhaust_special_execution_budget(self):
        import special_runtime
        from types import SimpleNamespace
        from unittest.mock import patch
        self.prepare('gap');clock=time.monotonic;offset=[0.]
        waiting=special_runtime.wait_special_ready
        def recovered(*args,**kw):
            result=waiting(*args,**kw);offset[0]+=500.;return result
        with patch.object(special_runtime,'time',SimpleNamespace(monotonic=lambda:clock()+offset[0])),patch.object(special_runtime,'wait_special_ready',side_effect=recovered):
            self.assertEqual(self.n.run_goal(),'arrived_stopped')
        self.assertIsNone(self.paused_at);self.assertEqual(self.n.transport.current,self.identity)
    def test_final_stop_drift_reopens_same_terminal_target(self):
        self.prepare('drift');self.assertEqual(self.n.run_goal(),'arrived_stopped')
        self.assertTrue(self.drifted);self.assertEqual(self.n.transport.current,self.identity)
        self.assertIsNone(self.paused_at)
    def test_new_special_goal_after_ended_normal_task_has_new_output_identity(self):
        old=self.n.transport.current;self.n.end_output(old)
        self.start_special(.23);new=self.n.transport.current
        self.assertNotEqual(old,new)
        self.n.transport.send((.01,0.,0.),new)
        self.assertFalse(old==self.n.transport.last_identity)
    def test_first_frame_after_input_recovery_keeps_jump_check(self):
        from special_runtime import wait_special_ready
        from special_segments import StraightChain
        self.start_special(.23);chain=StraightChain(self.position,self.position,self.n.waypoints.route.direct_points())
        chain.previous_sample=(self.n.now(),self.position.copy(),self.position.copy())
        mp,od,stamp=wait_special_ready(self.n,self.n.seen,chain)
        with self.assertRaisesRegex(ValueError,'localization_jump'):
            chain.step(mp+np.array([.23,0,0]),od+np.array([.23,0,0]),stamp+.02,stamp+.02,.02,[0,0,0])

if __name__=='__main__':unittest.main()
