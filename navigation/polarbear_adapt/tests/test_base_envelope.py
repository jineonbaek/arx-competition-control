import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
import chassis_filter


class BaseEnvelopeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.root=Path(self.tmp.name)
        self.wkt=self.root/'base.wkt'
        self.wkt.write_text('POLYGON ((-.32 -.32,.32 -.32,.32 .32,-.32 .32,-.32 -.32))')
        self.cfg=self.root/'mask.json'
        self.cfg.write_text(json.dumps({'enabled':True,'wkt_file':str(self.wkt)}))
        self.model=self.root/'mesh.npz'
        np.savez(self.model,rigid=np.array([[0.,0.,.1],[.005,0.,.1]]),pitch=.005)
        self.old=getattr(chassis_filter,'BASE_ENVELOPE_CONFIG',None)
        chassis_filter.BASE_ENVELOPE_CONFIG=self.cfg

    def tearDown(self):
        chassis_filter.BASE_ENVELOPE_CONFIG=self.old
        self.tmp.cleanup()

    def test_all_heights_inside_base_are_self_returns(self):
        points=np.array([[.1,.1,.7],[.1,.1,1.5],[.1,.1,-.1],[-.32,0.,.8]])
        self.assertTrue(chassis_filter.ChassisFilter(self.model).contains(points).all())

    def envelope(self):
        from base_envelope import BaseEnvelope
        return BaseEnvelope.load(self.cfg)

    def test_outside_base_and_nonfinite_are_not_masked(self):
        p=np.array([[.321,0.,.6],[0.,.321,.6],[0.,0.,float('nan')],[.6,.6,.6]])
        self.assertFalse(self.envelope().mask(p).any())

    def test_planning_copy_frees_known_and_unknown_inside_only(self):
        grid=np.full((10,10),100,dtype=np.int8)
        grid[5,5]=-1
        expected=grid.copy()
        expected[2:8,2:8]=0
        actual=self.envelope().planning_copy(grid,[-.5,-.5],.1,(0.,0.,0.))
        np.testing.assert_array_equal(actual,expected)
        self.assertEqual(grid[5,5],-1)
        self.assertEqual(grid[4,4],100)

    def test_boundary_crossing_cells_are_preserved(self):
        grid=np.full((10,10),100,dtype=np.int8)
        actual=self.envelope().planning_copy(grid,[-.5,-.5],.1,(0.,0.,0.))
        self.assertEqual(actual[5,8],100)  # [.3,.4] crosses the .32 boundary
        self.assertEqual(actual[5,7],0)

    def test_move_and_lost_pose_restore_original_region(self):
        grid=np.full((20,20),100,dtype=np.int8)
        mask=self.envelope()
        first=mask.planning_copy(grid,[-1.,-1.],.1,(0.,0.,0.))
        moved=mask.planning_copy(grid,[-1.,-1.],.1,(.6,0.,np.pi/2))
        self.assertEqual(first[10,10],0)
        self.assertEqual(moved[10,10],100)
        np.testing.assert_array_equal(mask.planning_copy(grid,[-1.,-1.],.1,None),grid)


if __name__=='__main__':unittest.main()
