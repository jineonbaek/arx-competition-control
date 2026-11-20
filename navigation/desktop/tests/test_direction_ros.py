import sys,math,unittest
from types import SimpleNamespace as S
import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from rclpy.clock import ClockType
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'polarbear_adapt'))
from dual_origin_grid import DualOriginMapping
class Direction(unittest.TestCase):
 def fixture(self,x=2.,y=3.,yaw=1.2):
  msg=TransformStamped();msg.header.stamp=Time(seconds=10).to_msg();msg.transform.translation.x=x;msg.transform.translation.y=y
  msg.transform.rotation.z=math.sin(yaw/2);msg.transform.rotation.w=math.cos(yaw/2);sent=[]
  return S(alignment=None,resume_pose=None,heading_pose=np.array([2.,3.,1.2]),tf=S(lookup_transform=lambda *a:msg),get_clock=lambda:S(now=lambda:Time(seconds=10.1,clock_type=ClockType.ROS_TIME)),broadcaster=S(sendTransform=sent.append)),sent
 def test_actual_mapper_sets_body_origin_and_heading(self):
  n,sent=self.fixture();self.assertTrue(DualOriginMapping.initialize_alignment(n));r,t=n.alignment
  np.testing.assert_allclose(r@np.array([2,3])+t,0,atol=1e-12)
  self.assertAlmostEqual(math.atan2(r[1,0],r[0,0])+1.2,0);self.assertEqual(len(sent),1)
 def test_actual_mapper_rejects_move_after_capture(self):
  n,sent=self.fixture(x=2.1)
  with self.assertRaisesRegex(ValueError,'Vehicle moved'):DualOriginMapping.initialize_alignment(n)
  self.assertFalse(sent)
if __name__=='__main__':unittest.main()
