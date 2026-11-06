from pathlib import Path
import importlib.util,sys,unittest
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))

class BatteryFeatureTests(unittest.TestCase):
 def module(self):
  self.assertIsNotNone(importlib.util.find_spec('battery_features'),'Unified feature association missing')
  import battery_features
  return battery_features
 def test_each_visible_slot_is_assigned_to_its_battery_once(self):
  m=self.module();im=np.full((480,640,3),35,np.uint8);boxes=[]
  for x in [180,430]:
   cv2.rectangle(im,(x-55,90),(x+55,320),(255,255,255),-1);cv2.rectangle(im,(x-20,280),(x+20,294),(100,100,100),-1)
   boxes.append({'xyxy':[x-55,90,x+55,320],'partial':False})
  holes=m.associate_holes(im,boxes)
  self.assertEqual(len(holes),2);self.assertEqual({h['battery_index'] for h in holes},{0,1})
  self.assertTrue(all(h['evidence']=='visible_image_slot' and h['xyz_m'] is None for h in holes))
 def test_hidden_slot_is_not_invented_from_body_or_model(self):
  m=self.module();im=np.full((480,640,3),35,np.uint8);cv2.rectangle(im,(120,90),(240,320),(255,255,255),-1)
  self.assertEqual(m.associate_holes(im,[{'xyxy':[120,90,240,320],'partial':False}]),[])
 def test_other_white_object_slot_outside_confirmed_battery_is_not_assigned(self):
  m=self.module();im=np.full((480,640,3),35,np.uint8)
  cv2.rectangle(im,(350,90),(500,320),(255,255,255),-1);cv2.rectangle(im,(390,280),(430,294),(100,100,100),-1)
  self.assertEqual(m.associate_holes(im,[{'xyxy':[100,90,240,320],'partial':False}]),[])

 def test_body_side_seam_is_not_counted_as_second_handle_hole(self):
  m=self.module();im=np.full((480,640,3),35,np.uint8)
  cv2.rectangle(im,(125,90),(235,320),(255,255,255),-1)
  cv2.rectangle(im,(160,280),(200,294),(100,100,100),-1)
  cv2.rectangle(im,(220,180),(227,202),(100,100,100),-1)
  holes=m.associate_holes(im,[{'xyxy':[125,90,235,320],'partial':False}])
  self.assertEqual(len(holes),1)
  self.assertGreater(holes[0]['center_px'][1],270)

 def test_depth_is_hidden_when_its_hole_is_not_jointly_confirmed(self):
  m=self.module();source={'detection':{'hole_px':[100,100]},'location':{'status':'surface_valid','bar_surface_xyz_m':[.1,.2,.6],'robot_xyz_m':None}}
  missing=m.bind_measurement(source,[])
  self.assertIsNone(missing['location']['bar_surface_xyz_m'])
  self.assertEqual(source['location']['bar_surface_xyz_m'],[.1,.2,.6])
  other=m.bind_measurement(source,[{'center_px':[300,100]}])
  self.assertIsNone(other['location']['bar_surface_xyz_m'])
 def test_matching_observed_hole_keeps_its_own_surface_depth(self):
  m=self.module();source={'detection':{'hole_px':[100,100]},'location':{'status':'surface_valid','bar_surface_xyz_m':[.1,.2,.6],'robot_xyz_m':None}}
  matched=m.bind_measurement(source,[{'center_px':[101,100]}])
  self.assertEqual(matched['location']['bar_surface_xyz_m'],[.1,.2,.6])
  self.assertTrue(matched['joint_association_valid'])

if __name__=='__main__':unittest.main()
