"""Explicit plant assumptions, independent of the controller; never a hardware claim."""
import unittest,os
import special_segments as current
from special_delayed_plant import simulate,module
from pathlib import Path
controller=module(Path(os.environ['BASELINE_SPECIAL']), 'baseline_under_test') if os.getenv('BASELINE_SPECIAL') else current
class DelayedPlant(unittest.TestCase):
 def tearDown(self):controller.NORMAL_SDK=.6
 def check(self,cap,delay,tau):
  result=simulate(controller,cap=cap,delay=delay,tau=tau,new=True)
  self.assertTrue(result['arrived'],result);self.assertLess(result['max_overshoot'],.08,result)
  self.assertLessEqual(result['reversals'],1,result);self.assertLess(result['straight_lateral_rms'],.003,result)
  self.assertLessEqual(result['max_sdk'],cap+.0001,result)
 def test_normal_with_200ms_delay(self):self.check(.6,.2,.15)
 def test_normal_with_300ms_delay_and_longer_coast(self):self.check(.6,.3,.25)
 def test_full_request_with_200ms_delay_retains_effort_envelope(self):self.check(1.,.2,.15)
 def test_full_request_with_300ms_delay_does_not_orbit(self):self.check(1.,.3,.25)
if __name__=='__main__':unittest.main()
