import unittest,struct
from readonly_encos_query import query_request,validate_write,decode_reply
class EncosBoundary(unittest.TestCase):
 def test_documented_position_request(self):
  self.assertEqual(query_request('/dev/arxcan1',1,1),b't0012E101\r')
 def test_unknown_ids_and_control_packets_rejected(self):
  with self.assertRaises(ValueError):query_request('/dev/arxcan1',3,1)
  for command in [b't0012C001\r',b't00180000000000000000\r',b't7FF400010003\r',b't0012E105\r']:
   with self.assertRaises(ValueError):validate_write('/dev/arxcan1',command)
 def test_position_reply_is_big_endian_degrees(self):
  frame={'can_id':1,'data_hex':(b'\xa0\x01'+struct.pack('>f',90.0)).hex()}
  decoded=decode_reply(frame,1,1)
  self.assertAlmostEqual(decoded['position_deg'],90.0)
if __name__=='__main__':unittest.main()
