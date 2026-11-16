"""Real supervisor: 500s parked replan is not 500s commanded execution."""
import time,unittest
from unittest.mock import patch
import rclpy
import test_retained_faults_ros as f

class LongWait(unittest.TestCase):
 setUpClass=classmethod(lambda cls:rclpy.init())
 tearDownClass=classmethod(lambda cls:rclpy.shutdown())
 setUp=f.RetainedFaults.setUp;tearDown=f.RetainedFaults.tearDown
 refresh=f.RetainedFaults.refresh;add=f.RetainedFaults.add;prepare=f.RetainedFaults.prepare
 def test_blocked_wait_longer_than_execution_limit_still_replans_and_arrives(self):
  self.prepare('long_replan');self.n.hold_for_replan('path_withdrawn')
  clock=time.monotonic;offset=[0.];refresh=self.refresh
  def feed():
   if not offset[0] and self.n.recovery is not None:
    offset[0]=500.;self.started+=500. # fixture watchdog, never the driver budget
   refresh()
  self.refresh=feed
  with patch('time.monotonic',lambda:clock()+offset[0]):
   with self.n.transport.task_scope(self.identity):self.assertEqual(self.n.run_goal(),'arrived_stopped')
  self.assertFalse(self.paused);self.assertEqual(self.n.transport.current,self.identity)
