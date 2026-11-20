import unittest
from edge_level_chain import assess_change
class Tests(unittest.TestCase):
 def test_parallel_requires_measured_angle(self):self.assertEqual(assess_change(-3.4,-.5,{'status':'image_roll_step_arrived'}),'parallel')
 def test_progress_after_tracking_timeout_is_measured_not_assumed(self):
  r={'status':'held','reason':'Measured arrival timeout'}
  self.assertEqual(assess_change(-3.4,-2.6,r),'continue')
  self.assertEqual(assess_change(-3.4,-3.3,r),'hold_no_progress')
 def test_other_failure_never_retried(self):self.assertEqual(assess_change(-3.4,-.5,{'status':'held','reason':'Body changed'}),'hold_execution_failure')
 def test_divergence_stops(self):self.assertEqual(assess_change(-3.4,-4.,{'status':'image_roll_step_arrived'}),'hold_diverging')
 def test_no_motion_claim_without_outcome(self):self.assertEqual(assess_change(-3.4,-.5,{'status':'planned'}),'hold_execution_failure')
if __name__=='__main__':unittest.main()

class MeasurementTests(unittest.TestCase):
 def row(self,i,angle):return dict(sequence=i,captured_at=i*.2,angle=angle,q=[0.]*6,height=7.4)
 def test_isolated_outliers_rejected(self):
  from edge_level_chain import stable_measurement
  rows=[self.row(i,v) for i,v in enumerate([-2.,-2.02,-2.01,-2.0,-1.98,-2.03,-2.01,5.,6.])]
  r=stable_measurement(rows);self.assertIsNotNone(r);self.assertEqual(len(r['frames']),7);self.assertLess(abs(r['angle_deg']+2),.03)
 def test_ambiguous_two_clusters_rejected(self):
  from edge_level_chain import stable_measurement
  self.assertIsNone(stable_measurement([self.row(i,v) for i,v in enumerate([-2]*5+[2]*4)]))
 def test_moving_arm_not_settled(self):
  from edge_level_chain import stable_measurement
  rows=[self.row(i,-2) for i in range(7)];rows[-1]['q'][0]=.1
  self.assertIsNone(stable_measurement(rows))

class EmpiricalCommandTests(unittest.TestCase):
 def test_command_is_bounded_and_signed(self):
  from edge_level_chain import empirical_command
  self.assertEqual(empirical_command(-3,.44),2.)
  self.assertEqual(empirical_command(3,.44),-2.)
  self.assertAlmostEqual(empirical_command(-.8,.5),1.6)
 def test_invalid_gain_rejected(self):
  from edge_level_chain import empirical_command
  for gain in (0,.01,10,float('nan')):
   with self.assertRaises(ValueError):empirical_command(-2,gain)
