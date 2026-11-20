import unittest,copy
from loaded_coarse import validate_start,arm_command
class Tests(unittest.TestCase):
 def setUp(self):
  self.s=dict(execution_enabled=True,body=dict(fresh=True,height=7.5),arms={'right':dict(fresh=True,joints=[0.]*6,gripper=-1.7),'left':dict(fresh=True,joints=[0.]*6)},motion_reserved=False,competition_reserved=False,chassis_active=False)
  self.f=dict(image_time=100.,joints=[0.]*6,lift_rad=7.5)
 def test_unchanged_state_valid(self):validate_start(self.s,self.f,[[0.]*6,[.01]*6],101.)
 def test_wrong_height_rejected(self):
  self.s['body']['height']=9.7
  with self.assertRaises(ValueError):validate_start(self.s,self.f,[[0.]*6,[.01]*6],101.)
 def test_stale_or_repositioned_rejected(self):
  with self.assertRaises(ValueError):validate_start(self.s,self.f,[[0.]*6,[.01]*6],111.)
  self.s['arms']['right']['joints'][0]=.1
  with self.assertRaises(ValueError):validate_start(self.s,self.f,[[0.]*6,[.01]*6],101.)
 def test_other_owner_rejected(self):
  self.s['motion_reserved']=True
  with self.assertRaises(ValueError):validate_start(self.s,self.f,[[0.]*6,[.01]*6],101.)
 def test_no_gripper_or_other_motor_fields(self):
  self.assertEqual(set(arm_command([0.]*6)),{'type','arm','joints'})
  with self.assertRaises(ValueError):arm_command([float('nan')]*6)
if __name__=='__main__':unittest.main()

class VerticalTargetTests(unittest.TestCase):
 def test_four_mm_only_changes_z(self):
  import numpy as np
  from loaded_coarse import vertical_target
  pose=np.eye(4);pose[:3,3]=[.3,-.1,.6];before=pose.copy();goal=vertical_target(pose,4.)
  np.testing.assert_array_equal(pose,before)
  np.testing.assert_allclose(goal[:3,3],[.3,-.1,.604]);np.testing.assert_array_equal(goal[:3,:3],pose[:3,:3])
 def test_rejects_excess_or_nonfinite_step(self):
  import numpy as np
  from loaded_coarse import vertical_target
  for value in (-1,0,6,float('nan')):
   with self.assertRaises(ValueError):vertical_target(np.eye(4),value)

class LateralTargetTests(unittest.TestCase):
 def test_negative_five_mm_only_changes_world_y(self):
  import numpy as np
  from loaded_coarse import lateral_target
  pose=np.eye(4);pose[:3,3]=[.3,-.1,.6];before=pose.copy();goal=lateral_target(pose,-5.)
  np.testing.assert_array_equal(pose,before)
  np.testing.assert_allclose(goal[:3,3],[.3,-.105,.6]);np.testing.assert_array_equal(goal[:3,:3],pose[:3,:3])
 def test_rejects_zero_excess_or_nonfinite_step(self):
  import numpy as np
  from loaded_coarse import lateral_target
  for value in (-5.01,0,5.01,float('nan')):
   with self.assertRaises(ValueError):lateral_target(np.eye(4),value)

class CameraRollTests(unittest.TestCase):
 def test_roll_preserves_tcp_position_and_cancels_image_line_error(self):
  import numpy as np
  from loaded_coarse import camera_roll_target
  pose=np.eye(4);pose[:3,3]=[.4,-.1,.6];camera=np.eye(4)
  goal=camera_roll_target(pose,camera,1.)
  np.testing.assert_array_equal(goal[:3,3],pose[:3,3])
  ray=np.array([1.,0.,1.]);rotated=goal[:3,:3].T@ray
  angle=np.degrees(np.arctan2(rotated[1],rotated[0]));self.assertAlmostEqual(angle,-1.)
 def test_reject_large_rotation(self):
  import numpy as np
  from loaded_coarse import camera_roll_target
  with self.assertRaises(ValueError):camera_roll_target(np.eye(4),np.eye(4),3.)
 def test_roll_up_target_preserves_xy_and_adds_world_z(self):
  import numpy as np
  from loaded_coarse import roll_up_target
  pose=np.eye(4);pose[:3,3]=[.4,-.1,.6]
  goal=roll_up_target(pose,np.eye(4),1.,5.)
  np.testing.assert_allclose(goal[:3,3],[.4,-.1,.605])
  self.assertAlmostEqual(float(np.degrees(np.arctan2(goal[1,0],goal[0,0]))),1.)

class AxisStepTests(unittest.TestCase):
 def test_bounded_step_moves_tool_x_toward_horizontal_axis_without_translation(self):
  import numpy as np
  from scipy.spatial.transform import Rotation
  from loaded_coarse import axis_step_target
  pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('y',5,degrees=True).as_matrix();pose[:3,3]=[.4,-.1,.6]
  goal=axis_step_target(pose,[1,0,0],2.2)
  self.assertAlmostEqual(float(np.degrees(np.arccos(np.clip(goal[:3,0]@[1,0,0],-1,1)))),2.8,places=4)
  np.testing.assert_array_equal(goal[:3,3],pose[:3,3])
 def test_rejects_large_step_or_vertical_axis(self):
  import numpy as np
  from loaded_coarse import axis_step_target
  with self.assertRaises(ValueError):axis_step_target(np.eye(4),[1,0,0],3.)
  with self.assertRaises(ValueError):axis_step_target(np.eye(4),[1,0,.1],2.)

class RetainedHoldTests(unittest.TestCase):
 def test_j6_update_keeps_other_targets(self):
  import numpy as np
  from loaded_coarse import joint6_goal
  q=np.array([0.,1.,.3,.9,.8,-1.5]);hold=q.copy();hold[3]+=.01
  goal=joint6_goal(q,2.,hold);np.testing.assert_array_equal(goal[:5],hold[:5]);self.assertAlmostEqual(goal[5],q[5]+np.radians(2))
 def test_rejects_large_reference_jump(self):
  import numpy as np
  from loaded_coarse import joint6_goal
  q=np.zeros(6);hold=q.copy();hold[0]=.2
  with self.assertRaises(ValueError):joint6_goal(q,1.,hold)
