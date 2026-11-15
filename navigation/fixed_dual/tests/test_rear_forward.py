import unittest
import numpy as np
from rear_sector import rear_forward_points,mask_rear_forward_scan,rear_points
class SectorTests(unittest.TestCase):
 def test_rear_native_minus_y_is_vehicle_forward(self):
  p=np.array([[0,-1,0],[0,-1,5],[0,1,0],[1,0,0],[-1,0,0]],float)
  np.testing.assert_array_equal(rear_forward_points(p),[True,True,False,False,False])
 def test_70_degree_symmetric_closed_boundary(self):
  angles=np.deg2rad([-125.001,-125,-124.999,-90,-55.001,-55,-54.999,60,120,180])
  p=np.c_[np.cos(angles),np.sin(angles),np.ones(len(angles))]
  np.testing.assert_array_equal(rear_forward_points(p),[False,True,True,True,True,True,False,False,False,False])
 def test_criterion_is_direction_from_sensor_not_base_origin(self):
  # Optical origin offset must not rotate the sector towards C_base.
  p=np.array([[0.,-.01,0],[0.,-.1,0],[0.,-2.,0]])
  self.assertTrue(rear_forward_points(p).all())
 def test_scan_mask_is_nan_never_inf_or_zero_and_preserves_outside(self):
  angles=np.deg2rad(np.arange(-180,180,dtype=np.float32));v=np.arange(720,1080,dtype=float)
  out=np.array(mask_rear_forward_scan(v,float(angles[0]),float(np.float32(np.pi/180))))
  expected=(np.arange(-180,180)>=-125)&(np.arange(-180,180)<=-55)
  self.assertTrue(np.isnan(out[expected]).all());np.testing.assert_array_equal(out[~expected],v[~expected])
 def test_old_outer_wedges_are_now_retained(self):
  angles=np.deg2rad([-150,-140,-126,-124,-90,-56,-54,-40,-30])
  p=np.c_[np.cos(angles),np.sin(angles),np.full(len(angles),.4)]
  np.testing.assert_array_equal(rear_forward_points(p),[False,False,False,True,True,True,False,False,False])
 def test_wrapped_positive_scan_angles_have_identical_mask(self):
  angles=np.arange(180,360,dtype=float);v=np.ones(len(angles))
  out=np.asarray(mask_rear_forward_scan(v,np.deg2rad(180),np.deg2rad(1)))
  expected=(angles>=235)&(angles<=305)
  np.testing.assert_array_equal(np.isnan(out),expected)
  np.testing.assert_array_equal(out[~expected],v[~expected])
 def test_front_rear85_is_unchanged(self):
  angles=np.deg2rad([0,137.4,137.5,180,-137.5,-137.4])
  np.testing.assert_array_equal(rear_points(np.c_[np.cos(angles),np.sin(angles),np.zeros(6)]),[False,False,True,True,True,False])
if __name__=='__main__':unittest.main()
