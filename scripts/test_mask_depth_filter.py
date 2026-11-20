import unittest
import numpy as np
from mask_depth_filter import filter_seeded_mask,validate_seeded_pose

class DepthMaskTest(unittest.TestCase):
    def test_excludes_connected_hardware_outside_battery(self):
        xyz=np.zeros((20,20,3),float);xyz[:,:,:]=[0,0,1.]
        xyz[:,15:,0]=.3
        mask=np.ones((20,20),np.uint8)*255
        transform=np.eye(4);transform[2,3]=1.
        result=filter_seeded_mask(mask,xyz,np.eye(4),transform,[[-.05,-.05,-.05],[.05,.05,.05]])
        self.assertEqual(np.count_nonzero(result[:,:15]),300)
        self.assertEqual(np.count_nonzero(result[:,15:]),0)
    def test_bad_seed_does_not_fabricate_confidence(self):
        xyz=np.zeros((20,20,3),float);xyz[:,:,:]=[.5,0,1.]
        mask=np.ones((20,20),np.uint8)*255;transform=np.eye(4);transform[2,3]=1.
        with self.assertRaises(ValueError):
            filter_seeded_mask(mask,xyz,np.eye(4),transform,[[-.05,-.05,-.05],[.05,.05,.05]])

    def test_rejects_large_pose_flip_despite_good_depth_quality(self):
        seed=np.eye(4);estimated=np.eye(4);estimated[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        with self.assertRaises(ValueError):validate_seeded_pose(seed,estimated)
        validate_seeded_pose(seed,seed.copy())
