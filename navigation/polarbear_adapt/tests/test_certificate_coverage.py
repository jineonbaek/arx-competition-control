import unittest
import numpy as np
from path_audit import AuditGrid
from async_path_audit import CoverageGrid,Certificate

class CoverageTests(unittest.TestCase):
 def grid(self):return AuditGrid(np.zeros((20,20),np.int8),[-1.,-1.],.1,[[.4,0],[0,.4],[-.4,0],[0,-.4]])
 def cert(self,g,pose=(0.,0.,0.)):
  covered=CoverageGrid(g);result=AuditGrid.along(covered,[pose]);return Certificate(covered,result)
 def test_changed_corner_outside_actual_swept_volume_does_not_cancel_certificate(self):
  g=self.grid();c=self.cert(g);g.grid[5,5]=100
  self.assertIsNone(g.inspect([0.,0.,0.])['first_cell'])
  self.assertTrue(c.compatible(g),'An unrelated bounding-box corner must not force a new audit')
 def test_new_obstacle_or_unknown_in_previously_free_touched_cell_invalidates(self):
  for value in (100,-1):
   g=self.grid();c=self.cert(g);g.grid[11,11]=value
   self.assertIsNotNone(g.inspect([0.,0.,0.])['first_cell']);self.assertFalse(c.compatible(g))
 def test_every_collision_cell_invalidates_including_grazing_rotated_cells(self):
  for yaw in (0.,.37,1.1):
   g=self.grid();pose=[0.,0.,yaw];c=self.cert(g,pose)
   for y in range(20):
    for x in range(20):
     g.grid[y,x]=100
     if g.inspect(pose)['first_cell'] is not None:self.assertFalse(c.compatible(g),(yaw,x,y))
     g.grid[y,x]=0
 def test_expanded_rotation_margin_is_also_certified(self):
  g=self.grid();covered=CoverageGrid(g);vertices=g._vertices([0.,0.,.37]);result=covered._inspect_vertices(vertices,margin=.08);c=Certificate(covered,result)
  for y in range(20):
   for x in range(20):
    g.grid[y,x]=100
    if g._inspect_vertices(vertices,margin=.08)['first_cell'] is not None:self.assertFalse(c.compatible(g),(x,y))
    g.grid[y,x]=0
 def test_footprint_change_always_invalidates(self):
  g=self.grid();c=self.cert(g);g.poly*=1.01;self.assertFalse(c.compatible(g))
if __name__=='__main__':unittest.main()
