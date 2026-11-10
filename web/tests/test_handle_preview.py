import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from handle_preview import handle_line


class HandleLineTests(unittest.TestCase):
    def setUp(self):
        y,z=np.meshgrid(np.linspace(-.02,.02,48),np.linspace(-.01,.01,24))
        self.xyz=np.stack([y,z,np.ones_like(y)*.3],axis=-1)
        self.t=np.eye(4)
        self.t[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        self.t[:3,3]=[.2,-.25,.7]
        self.roi=[0,0,48,24]

    def test_center_hole_and_some_background(self):
        self.xyz[9:15,17:31]=0
        self.xyz[:2,:,2]=.8
        result=handle_line(self.xyz,self.t,self.roi,50)
        np.testing.assert_allclose(result['feature_mm'],[500,-250,700],atol=1)
        np.testing.assert_allclose(np.subtract(result['feature_mm'],result['pre_mm']),[50,0,0])

    def test_ambiguous_or_sparse_depth(self):
        self.xyz[:12,:,2]=.8
        with self.assertRaises(ValueError):handle_line(self.xyz,self.t,self.roi,50)
        self.xyz[:]=0
        with self.assertRaises(ValueError):handle_line(self.xyz,self.t,self.roi,50)
