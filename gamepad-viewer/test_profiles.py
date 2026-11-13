import unittest
from profiles import select_profile, UnknownMode, logical_axes, lt_pressed, raw_normalized
NS_AX=[0,1,3,4,16,17]
NS_BT=[304,305,307,308,309,310,311,312,313,314,315,316,317,318]
XI_AX=[0,1,2,3,4,5,16,17]
XI_BT=[304,305,307,308,310,311,314,315,316,317,318]
class ProfileTests(unittest.TestCase):
 def test_selects_only_known_complete_signatures(self):
  self.assertEqual(select_profile(0x057e,0x2009,NS_AX,NS_BT)['mode'],'NS')
  self.assertEqual(select_profile(0x20bc,0x5128,XI_AX,XI_BT)['mode'],'XINPUT')
  with self.assertRaises(UnknownMode):select_profile(0x1234,0x5678,XI_AX,XI_BT)
  with self.assertRaises(UnknownMode):select_profile(0x20bc,0x5128,NS_AX,XI_BT)
 def test_xinput_uses_right_stick_3_4_not_lt_axis(self):
  p=select_profile(0x20bc,0x5128,XI_AX,XI_BT)
  a=[0,0,-32767,32767,-32767,-32767,0,0]
  self.assertEqual(logical_axes(p,a),[0,0,1,-1,0,0])
  self.assertFalse(lt_pressed(p,a,[0]*11))
  a[2]=32767
  self.assertTrue(lt_pressed(p,a,[0]*11))
  self.assertEqual(logical_axes(p,a),[0,0,1,-1,0,0])
 def test_ns_original_mapping_kept(self):
  p=select_profile(0x057e,0x2009,NS_AX,NS_BT)
  a=[-16380,16379,-16380,16379,0,0];b=[0]*14
  self.assertEqual(logical_axes(p,a),[0]*6)
  b[7]=1;self.assertTrue(lt_pressed(p,a,b))
 def test_triggers_normalize_to_zero_released(self):
  p=select_profile(0x20bc,0x5128,XI_AX,XI_BT)
  self.assertEqual(raw_normalized(p,2,-32767),0)
  self.assertEqual(raw_normalized(p,2,32767),1)
if __name__=='__main__':unittest.main()
