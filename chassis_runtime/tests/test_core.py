import unittest
from arx_chassis.core import Core, Rejected
class CoreTests(unittest.TestCase):
 def setUp(self):
  self.now=99.;self.outputs=[]
  self.c=Core(self.outputs.append,wall=lambda:self.now,clock=lambda:self.now)
  self.now=100.;self.c.driver=('driver',True,self.now);self.c.bound_instance=self.c.instance
  self.a=self.c.register('manual');self.b=self.c.register('navigation')
 def msg(self,client,seq=1,**kw):
  d=dict(client_id=client,scheduler_instance=self.c.instance,operation='move',kind='chassis',source_id='web-test',task_id='keyboard',generation=1,seq=seq,issued_at=self.now,ttl_s=1.,sdk=[.1,0.,0.]);d.update(kw);return d
 def test_browser_one_second_preserved(self):
  self.c.command(self.msg(self.a,issued_at=99.8));self.assertAlmostEqual(self.outputs[-1]['ttl_s'],.8)
  self.now=100.79;self.c.driver=('driver',True,self.now);self.c.tick();self.assertIsNotNone(self.c.scheduler.active)
  self.now=100.8;self.c.tick();self.assertIsNone(self.c.scheduler.active)
 def test_profile_cannot_extend_navigation_lease(self):
  with self.assertRaises(Rejected):self.c.command(self.msg(self.b,kind='chassis.nav_sdk',ttl_s=1.001))
 def test_scoped_release_does_not_stop_other_writer(self):
  self.c.command(self.msg(self.a));self.c.command(self.msg(self.b,kind='chassis.nav_sdk',source_id='navigation-b',ttl_s=.3))
  self.c.command(self.msg(self.a,2,operation='end'));self.assertEqual(self.c.scheduler.active['client'],self.b)
 def test_old_instance_and_stale_packets_rejected(self):
  for kw in [dict(scheduler_instance='old'),dict(issued_at=98.),dict(issued_at=101.),dict(seq=0),dict(sdk=[float('nan'),0,0])]:
   with self.assertRaises((Rejected,ValueError)):self.c.command(self.msg(self.a,**kw))
 def test_driver_loss_and_new_driver_close_old_tasks(self):
  self.c.command(self.msg(self.a));self.now+=.3;self.c.tick();self.assertIsNone(self.c.scheduler.active)
  self.c.driver=('new',True,self.now);self.c.bound_instance=self.c.instance
  with self.assertRaises((Rejected,ValueError)):self.c.command(self.msg(self.a,2))
 def test_maintenance_owner_disconnect_expires(self):
  self.c.reserve(self.a);self.now+=2.1;self.c.tick();self.assertIsNone(self.c.maintenance_client)
 def test_expired_generation_cannot_resume(self):
  self.c.command(self.msg(self.a));self.now+=1.;self.c.driver=('driver',True,self.now);self.c.tick()
  with self.assertRaises((Rejected,ValueError)):self.c.command(self.msg(self.a,2))
 def test_duplicate_sequence_rejected_without_stop(self):
  self.c.command(self.msg(self.a))
  with self.assertRaises((Rejected,ValueError)):self.c.command(self.msg(self.a))
  self.assertEqual(self.c.scheduler.active['sdk'],[.1,0.,0.])
if __name__=='__main__':unittest.main()

class RegressionTests(CoreTests):
 def test_expired_maintenance_client_cannot_return_as_ordinary(self):
  self.c.reserve(self.a);self.now+=2.1;self.c.tick();self.c.driver=('driver',True,self.now)
  with self.assertRaises(ValueError):self.c.command(self.msg(self.a,generation=2))
 def test_positive_clock_skew_does_not_extend_navigation_ttl(self):
  self.c.command(self.msg(self.b,kind='chassis.nav_sdk',source_id='navigation-skew',issued_at=self.now+.09,ttl_s=.3))
  self.assertLessEqual(self.outputs[-1]['ttl_s'],.300001)
