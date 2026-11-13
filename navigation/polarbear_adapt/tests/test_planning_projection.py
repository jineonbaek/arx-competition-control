import unittest
from pathlib import Path
import numpy as np
import shapely
from chassis_projection import RigidProjection


class WholeCellTest(unittest.TestCase):
    def test_edges_hole_and_known_cells(self):
        # An interior hole not containing a cell corner still excludes the cell.
        shape=shapely.from_wkt('POLYGON ((0 0,3.8 0,3.8 2,0 2,0 0),(1.2 .2,1.4 .2,1.4 .4,1.2 .4,1.2 .2))')
        grid=np.full((4,5),-1,dtype=np.int8)
        grid[1,0]=100;grid[0,2]=0
        indices=RigidProjection(shape).unknown_cells(grid,(0.,0.),1.,(0.,0.,0.))
        self.assertEqual(set(indices),{0,6,7})
        self.assertEqual(grid[0,1],-1)
        self.assertEqual(grid[1,0],100)

    def test_nominal_shape_and_rotation(self):
        path=Path(__file__).resolve().parents[1]/'chassis_unknown/projection.wkt'
        shape=shapely.from_wkt(path.read_text())
        projection=RigidProjection(shape)
        grid=np.full((40,40),-1,dtype=np.int8);org=(-1.,-1.);res=.05
        for pose in [(0.,0.,0.),(.21,.17,.8)]:
            indices=projection.unknown_cells(grid,org,res,pose)
            self.assertGreater(len(indices),0)
            from shapely.affinity import rotate,translate
            world=translate(rotate(shape,pose[2],origin=(0,0),use_radians=True),*pose[:2])
            expected=set()
            for y,x in np.ndindex(grid.shape):
                bounds=(org[0]+x*res,org[1]+y*res,org[0]+(x+1)*res,org[1]+(y+1)*res)
                if world.covers(shapely.box(*bounds)):expected.add(y*40+x)
            self.assertEqual(set(indices),expected)
