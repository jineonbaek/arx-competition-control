import unittest
from readonly_motor_query import read_request,validate_serial_write
class QueryBoundary(unittest.TestCase):
 def test_exact_documented_read_frame(self):
  self.assertEqual(read_request('/dev/arxcan5',1,13),b't7FF80100330D00000000\r')
 def test_unknown_targets_and_registers_are_rejected(self):
  for args in [('/dev/arxcan5',2,13),('/dev/arxcan1',1,13),('/dev/arxcan5',1,0),('/dev/ttyACM0',1,13)]:
   with self.assertRaises(ValueError):read_request(*args)
 def test_all_control_and_register_write_frames_are_rejected(self):
  for data in [b't0018FFFFFFFFFFFFFFFC\r',b't00180000000000000000\r',b't7FF80100550D00000000\r',b't7FF80100AA0100000000\r',b't7FF80100CC0000000000\r']:
   with self.assertRaises(ValueError):validate_serial_write('/dev/arxcan5',data)
if __name__=='__main__':unittest.main()
