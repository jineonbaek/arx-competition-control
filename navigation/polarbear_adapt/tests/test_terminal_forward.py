import unittest
from terminal_forward import TerminalForward
class TerminalTests(unittest.TestCase):
 def permit(self):return dict(route_id='r',mission_id='m',point_id='p',authorized=True,sdk_forward=.3,duration_s=6.)
 def test_timer_and_identity(self):
  p=self.permit();f=TerminalForward(p,10.)
  self.assertEqual(f.command(p,15.9),(.3,0.,0.));self.assertEqual(f.command(p,16.),(0.,0.,0.))
  r=f.receipt(16.);self.assertTrue(r['output_ended']);self.assertEqual(r['duration_s'],6.)
 def test_denied_guard_and_old_mission_fail_closed(self):
  p=self.permit();f=TerminalForward(p,10.)
  for bad in (dict(p,authorized=False),dict(p,mission_id='old'),None,dict(p,duration_s=100.)):
   with self.assertRaises(ValueError):f.command(bad,11.)
 def test_no_early_receipt(self):
  f=TerminalForward(self.permit(),10.)
  with self.assertRaises(ValueError):f.receipt(15.)
