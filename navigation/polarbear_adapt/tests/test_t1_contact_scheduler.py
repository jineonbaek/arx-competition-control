"""Actual arbiter deadline/fence behavior, with injected clock and inert output."""
import unittest
from arx_chassis.scheduler import ChassisScheduler,InvalidChassis
class SchedulerDeadline(unittest.TestCase):
 def setUp(self):
  self.t=100.;self.outputs=[];self.s=ChassisScheduler(self.outputs.append,clock=lambda:self.t,wall=lambda:self.t)
 def send(self,sdk,seq=1,ttl=.2):
  m=dict(type='chassis.nav_sdk',source_id='nav',task_id='t1',generation=1,seq=seq,issued_at=self.t,ttl_s=ttl)
  return self.s.submit(m,'navigation-client',sdk)
 def test_contact_deadline_fences_motion_without_an_upper_layer_stop(self):
  self.send([.225,0,0]);self.t=100.199;self.s.tick();self.assertIsNotNone(self.s.active)
  self.t=100.201;self.s.tick();self.assertIsNone(self.s.active);self.assertEqual(self.outputs[-1]['sdk'],[0.,0.,0.])
  with self.assertRaises(InvalidChassis):self.send([.225,0,0],seq=2)
 def test_arrival_zero_does_not_fence_identity_before_contact(self):
  self.send([0.,0.,0.],ttl=.3);self.t=101.2;self.s.tick();self.send([.225,0,0],seq=2)
  self.assertEqual(self.s.active['sdk'],[.225,0,0]);self.assertAlmostEqual(self.s.active['deadline'],101.4)
if __name__=='__main__':unittest.main()
