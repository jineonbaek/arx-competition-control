import unittest,cv2,numpy as np
from d405_axis import estimate_long_axis,axis_consensus
class AxisTests(unittest.TestCase):
 def frame(self,angle=0.):
  h,w=240,320;rgb=np.full((h,w,3),220,np.uint8);cv2.rectangle(rgb,(40,20),(280,220),(20,20,20),-1)
  yy,xx=np.mgrid[:h,:w];x=(yy-20)*.0025;z=.4+(xx-160)*.0006+np.tan(np.radians(angle))*x;y=np.full_like(x,.2,dtype=float)
  return {"camera_id":"d405","rgb":rgb,"xyz":np.stack((x,y,z),-1),"base_from_color":np.eye(4)}
 def test_horizontal_body_axis(self):
  x=estimate_long_axis(self.frame());self.assertAlmostEqual(x["tilt_deg"],0.,delta=.2);self.assertFalse(x["executable"])
 def test_known_tilt(self):
  x=estimate_long_axis(self.frame(6.));self.assertAlmostEqual(x["tilt_deg"],6.,delta=.8)
 def test_missing_depth_fails(self):
  f=self.frame();f["xyz"][:]=0
  with self.assertRaises(ValueError):estimate_long_axis(f)
 def test_consensus_reports_repeatability_not_execution(self):
  frames=[]
  for i in range(3):
   f=self.frame(2.);f.update(image_time=10.+i*.1,arm_time=10.+i*.1,body_time=10.+i*.1,joints=[0.]*6,lift_rad=3.,calibration_sha256="a"*64);frames.append(f)
  r=axis_consensus(frames);self.assertAlmostEqual(r["tilt_deg"],2.,delta=.8);self.assertFalse(r["executable"]);self.assertFalse(r["physical_accuracy_verified"])
 def test_consensus_rejects_robot_movement_and_calibration_changes(self):
  frames=[]
  for i in range(3):
   f=self.frame();f.update(image_time=10.+i*.1,arm_time=10.+i*.1,body_time=10.+i*.1,joints=[0.]*6,lift_rad=3.,calibration_sha256="a"*64);frames.append(f)
  frames[1]["joints"]=[.1,0,0,0,0,0]
  with self.assertRaises(ValueError):axis_consensus(frames)
 def test_unknown_camera_fails(self):
  f=self.frame();f["camera_id"]="d435"
  with self.assertRaises(ValueError):estimate_long_axis(f)
if __name__=="__main__":unittest.main()
