import unittest,threading
from types import SimpleNamespace
from navigation_session import NavigationSession
class WebTerminal(unittest.TestCase):
 def session(self):
  n=object.__new__(NavigationSession);n.active=True;n.route_id='r';n.mission='m'
  n.prepared_definition=dict(points=[dict(id='p',type='special')]);n.terminal_context=dict(run_id='x',cancelled=threading.Event(),chassis_insertion=True)
  return n
 def test_guard_and_unchanged_policy(self):
  n=self.session();p=n.terminal_forward_permit()
  self.assertTrue(p['authorized']);self.assertEqual((p['sdk_forward'],p['duration_s']),(.3,6.))
  def broken():raise ValueError('gripper dropped')
  n.terminal_context['chassis_hold']=broken
  self.assertFalse(n.terminal_forward_permit()['authorized'])
 def test_completed_receipt_does_not_create_second_sender(self):
  n=self.session();p=n.terminal_forward_permit();n.terminal_receipt=dict(p,status='completed',output_ended=True)
  def forbidden(**kw):raise AssertionError('second sender created')
  r=n.finish_forward(n.terminal_context,transport_factory=forbidden)
  self.assertTrue(r['output_ended'])
  n.terminal_receipt['mission_id']='old'
  with self.assertRaises(ValueError):n.finish_forward(n.terminal_context,transport_factory=forbidden)
 def test_cancel_denies_permit(self):
  n=self.session();n.terminal_context['cancelled'].set();self.assertFalse(n.terminal_forward_permit()['authorized'])
