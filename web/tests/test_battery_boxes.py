import importlib.util
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))

class BatteryBoxesTests(unittest.TestCase):
 def module(self):
  self.assertIsNotNone(importlib.util.find_spec('battery_boxes'),'Whole battery postprocessor missing')
  import battery_boxes
  return battery_boxes
 def test_only_batteries_are_returned_with_clipped_coordinates(self):
  m=self.module()
  rows=[{'xyxy':[100,80,240,390],'score':.8,'class_id':0},{'xyxy':[300,100,500,300],'score':.9,'class_id':1},{'xyxy':[0,430,140,520],'score':.7,'class_id':0}]
  r=m.filter_boxes(rows,640,480)
  self.assertEqual(len(r),2)
  complete=next(x for x in r if x['xyxy'][0]==100);partial=next(x for x in r if x['xyxy'][0]==0)
  self.assertFalse(complete['partial']);self.assertTrue(partial['partial'])
  self.assertEqual(partial['xyxy'],[0,430,140,479])
  self.assertTrue(all(x['xyz_m'] is None for x in r))
 def test_empty_low_score_nan_and_zero_area_never_create_boxes(self):
  m=self.module()
  rows=[{'xyxy':[1,2,1,100],'score':.9,'class_id':0},{'xyxy':[1,2,100,200],'score':.1,'class_id':0},{'xyxy':[1,float('nan'),100,200],'score':.9,'class_id':0}]
  self.assertEqual(m.filter_boxes(rows,640,480),[])
  self.assertEqual(m.filter_boxes([],640,480),[])

 def test_weak_full_white_holder_match_is_not_promoted_to_complete_battery(self):
  m=self.module()
  rows=[{'xyxy':[77,70,296,207],'score':.32,'class_id':0},{'xyxy':[163,146,310,361],'score':.84,'class_id':0},{'xyxy':[0,145,134,364],'score':.28,'class_id':0}]
  boxes=m.filter_boxes(rows,640,480,min_score=.25)
  self.assertEqual(len(boxes),2)
  self.assertFalse(any(x['xyxy']==[77,70,296,207] for x in boxes))

if __name__=='__main__':unittest.main()
