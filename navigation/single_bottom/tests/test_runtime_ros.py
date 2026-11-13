"""Run on NUC with ROS_DOMAIN_ID=199; no robot/control topics."""
import json,tempfile,unittest
from pathlib import Path
import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg,CustomPoint
from runtime import Runtime
from core import upright_config,body_state

class RuntimeTests(unittest.TestCase):
 def test_filter_keeps_point_times_order_and_metadata(self):
  rclpy.init()
  try:
   with tempfile.TemporaryDirectory() as folder:
    folder=Path(folder);cfg={'bottom':upright_config()};mesh=folder/'mesh.npz';np.savez(mesh,rigid=np.array([cfg['bottom']['lidar_origin_m']]),head=np.empty((0,3)),nominal_head=np.eye(4),pitch=.005)
    node=Runtime('filter',cfg,mesh,folder)
    try:
     m=CustomMsg();m.header.stamp.sec=10;m.timebase=10000000000;m.lidar_id=3
     for i,x in enumerate([2.,0.,3.]):
      p=CustomPoint();p.x=x;p.y=0.;p.z=0.;p.offset_time=i*1000000;p.reflectivity=10+i;p.line=i;p.tag=16;m.points.append(p)
     m.point_num=3;node.filter(m)
     self.assertEqual(m.point_num,2);self.assertEqual(m.timebase,10000000000);self.assertEqual(m.lidar_id,3)
     self.assertEqual([(p.x,p.offset_time,p.reflectivity,p.line,p.tag) for p in m.points],[(2.,0,10,0,16),(3.,2000000,12,2,16)])
     node.filter(m);self.assertTrue(node.failed);self.assertEqual(node.count,1)
    finally:node.destroy_node()
  finally:rclpy.shutdown()

 def test_lever_covariance_is_positive_and_includes_angular_uncertainty(self):
  p,q,v,J,Jp=body_state([0,0,0],[0,0,0,1],np.zeros(6),[1,0,0]);C=J@np.eye(6)@J.T
  self.assertAlmostEqual(C[1,1],2.);self.assertAlmostEqual(C[2,2],2.);self.assertTrue(np.all(np.linalg.eigvalsh(C)>0));np.testing.assert_allclose(C,C.T)

if __name__=='__main__':unittest.main()
