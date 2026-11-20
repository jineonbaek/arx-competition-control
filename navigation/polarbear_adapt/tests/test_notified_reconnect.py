"""Reconnect only a provably unaccepted visit; never replay physical work."""
import unittest
import test_task_handoff as f
from task_handoff import HandoffHub,ProtocolError

class NotificationReconnect(unittest.TestCase):
 setUp=f.Handoffs.setUp;tearDown=f.Handoffs.tearDown
 begin=f.Handoffs.begin;event=f.Handoffs.event
 def expire(self):self.clock+=2.1;self.h.tick()
 def test_same_instance_recovers_original_unaccepted_notification(self):
  a=self.begin();self.expire();renewed=self.h.register(self.worker)
  self.assertEqual(renewed['token'],self.token)
  self.assertEqual(self.h.active['visit_id'],a['visit_id'])
  self.assertEqual(self.h.active['status'],'notified')
  self.assertFalse(self.h.active['navigation_ready']);self.assertFalse(self.h.active['control_released'])
  self.assertEqual(self.h.requests,{})
  msg=self.event(a,'accept');self.h.event(self.token,msg);self.h.event(self.token,msg)
  self.assertEqual(len(self.h.requests),1);self.assertEqual(self.h.active['status'],'running')
 def test_capability_change_cannot_recover_even_same_instance(self):
  self.begin();self.expire()
  with self.assertRaises(ProtocolError):self.h.register(dict(self.worker,point_ids=['point_2']))
  self.assertEqual(self.h.active['status'],'needs_attention')
 def test_accepted_job_is_still_uncertain_after_reconnect(self):
  a=self.begin();self.h.event(self.token,self.event(a,'accept'));self.expire();self.h.register(self.worker)
  self.assertEqual(self.h.active['status'],'needs_attention')
 def test_task_deadline_cannot_be_reset_by_reconnect(self):
  self.begin();self.clock+=601;self.h.tick();self.h.register(self.worker)
  self.assertEqual(self.h.active['status'],'needs_attention')
 def test_restart_never_infers_that_unaccepted_work_is_safe(self):
  self.begin();self.expire();h=HandoffHub(self.path,clock=lambda:self.clock)
  with self.assertRaises(ProtocolError):h.register(self.worker)
  self.assertEqual(h.active['error'],'navigation_restart')
 def test_operator_cancellation_cannot_be_undone_by_lease_reconnect(self):
  self.begin();self.expire();self.h.cancel();self.h.register(self.worker)
  self.assertEqual(self.h.active['status'],'cancelling')
