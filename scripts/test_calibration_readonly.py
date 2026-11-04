import unittest
from readonly_calibration_parameters import allowed_commands
class FixedReadBoundary(unittest.TestCase):
 def test_documented_zero_offset_queries_allowed(self):
  a=allowed_commands('/dev/arxcan1')
  self.assertIn(b't0012E126\r',a)
  self.assertIn(b't7FF80500333600000000\r',a)
 def test_no_enable_or_zero_or_parameter_write(self):
  for p in ['/dev/arxcan1','/dev/arxcan3','/dev/arxcan5']:
   a=allowed_commands(p)
   for c in [b't0018FFFFFFFFFFFFFFFC\r',b't0018FFFFFFFFFFFFFFFE\r',b't7FF80500553600000000\r',b't7FF400010003\r']:self.assertNotIn(c,a)
 def test_no_targets_beyond_fixed_ids(self):
  self.assertNotIn(b't0032E126\r',allowed_commands('/dev/arxcan1'))
if __name__=='__main__':unittest.main()
