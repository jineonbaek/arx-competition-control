import unittest,math
from blind_core import keep_point,filter_scan
class BlindTests(unittest.TestCase):
 def test_horizontal_optical_origin(self):
  self.assertFalse(keep_point([.1,.1,1.],.35))
  self.assertTrue(keep_point([.36,0.,0.],.35))
  self.assertFalse(keep_point([float('nan'),0,0],.35))
 def test_invalid_never_becomes_clearing_ray(self):
  r=filter_scan([.2,.35,1.,float('inf'),float('nan')],.35,8.)
  self.assertTrue(math.isnan(r[0]));self.assertEqual(r[1:3],[.35,1.]);self.assertTrue(all(math.isnan(x) for x in r[3:]))
if __name__=='__main__':unittest.main()
