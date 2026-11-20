import ast,sys,unittest,os
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from grid_visuals import cell_labels,merged_rectangles,geometry
class Visuals(unittest.TestCase):
 def roundtrip(self,a,costmap=False):
  labels=cell_labels(a.ravel(),a.shape[1],a.shape[0],costmap);rect=merged_rectangles(labels);out=np.zeros_like(labels)
  for group,x0,y0,x1,y1 in rect:
   self.assertTrue(x0<x1 and y0<y1);self.assertFalse(out[y0:y1,x0:x1].any());out[y0:y1,x0:x1]=group
  np.testing.assert_array_equal(labels,out);return labels,rect
 def test_random_exact_grid_coverage_and_unknown_holes(self):
  rng=np.random.default_rng(19)
  for _ in range(25):self.roundtrip(rng.choice([-1,0,1,98,99,100],size=(19,27)),True)
 def test_empty_map_clears_geometry(self):
  for width,height in ((0,0),(0,2)):
   lab=cell_labels([],width,height,True);rect=merged_rectangles(lab);self.assertFalse(rect);self.assertTrue(all(len(points)==0 for kind,points in geometry(lab,rect,.02).values()))
 def test_uniform_large_area_is_one_rectangle(self):
  lab,rect=self.roundtrip(np.zeros((500,500),np.int8));self.assertEqual(len(rect),1);self.assertEqual(geometry(lab,rect,.02)[1][0],'triangles');self.assertEqual(len(geometry(lab,rect,.02)[1][1]),6)
 def test_costmap_free_not_displayed_and_occupied_stays(self):
  labels,rect=self.roundtrip(np.array([[0,-1,100,99,1]],np.int8),True);np.testing.assert_array_equal(labels,[[0,0,2,3,4]])
 def test_checkerboard_uses_no_more_points_than_old_cubes(self):
  a=np.indices((40,40)).sum(axis=0)%2*100;labels,rect=self.roundtrip(a,True);kind,points=geometry(labels,rect,.02)[2];self.assertEqual(kind,'cubes');self.assertEqual(len(points),800)
 def test_triangles_preserve_metric_coordinates(self):
  labels,rect=self.roundtrip(np.full((3,3),100,np.int8));kind,points=geometry(labels,rect,.02)[2];self.assertEqual(kind,'triangles');self.assertEqual(np.max(points,axis=0).tolist(),[.06,.06]);self.assertEqual(np.min(points,axis=0).tolist(),[0.,0.])
class MappingInputs(unittest.TestCase):
 def test_both_mapping_scans_use_world_height_cropped_clouds(self):
  tree=ast.parse((ROOT/'navigation.launch.py').read_text());nodes=[]
  for f in ast.walk(tree):
   if not isinstance(f,ast.For) or not isinstance(f.target,ast.Tuple) or [x.id for x in f.target.elts if isinstance(x,ast.Name)]!=['name','lo','hi']:continue
   scope=dict(Node=lambda **kw:kw,nodes=nodes,sim=False,blind=.35,dual=True,os=os)
   exec(compile(ast.Module(body=[f],type_ignores=[]),'scan-input','exec'),scope)
  mapping=[n for n in nodes if n['name'].endswith('mapping_scan')];self.assertEqual(len(mapping),2)
  for n in mapping:
   self.assertIn(dict(n['remappings'])['cloud_in'],('/pb/bottom/nav_mark','/pb/head/nav_mark'));p=n['parameters'][0];self.assertEqual((p['min_height'],p['max_height']),(-8.,8.))
if __name__=='__main__':unittest.main()
