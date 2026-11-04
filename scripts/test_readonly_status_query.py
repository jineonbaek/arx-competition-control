import unittest
from readonly_status_query import status_request,validate_write,parse_frames
class StatusBoundary(unittest.TestCase):
 def test_status_requires_identity(self):
  with self.assertRaises(ValueError):status_request('/dev/arxcan5',1,False)
  self.assertEqual(status_request('/dev/arxcan5',1,True),b't7FF80100CC0000000000\r')
 def test_only_exact_query_commands_are_allowed(self):
  for command in [b't0018FFFFFFFFFFFFFFFC\r',b't0018FFFFFFFFFFFFFFFD\r',b't7FF8010055150000803F\r',b't7FF80100AA0100000000\r']:
   with self.assertRaises(ValueError):validate_write('/dev/arxcan5',command,True)
 def test_parse_real_version_reply(self):
  frames=parse_frames(b't00080100330D33303056\r')
  self.assertEqual(frames[0]['data_hex'],'0100330D33303056')
if __name__=='__main__':unittest.main()
