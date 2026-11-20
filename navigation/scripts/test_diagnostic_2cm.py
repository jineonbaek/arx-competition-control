"""Offline projection and launch descriptions only; never launch hardware."""
import os,sys,tempfile,subprocess,json,unittest,importlib.util
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from grid_core import project
from grid_core import save_grid
class DiagnosticResolution(unittest.TestCase):
 def test_full_extent_export_avoids_int64_temporaries_and_preserves_classes(self):
  import tracemalloc
  grid=np.full((8617,6263),-1,dtype=np.int8);grid[0,0]=0;grid[0,1]=100
  with tempfile.TemporaryDirectory() as d:
   tracemalloc.start()
   try:
    save_grid(Path(d)/'grid',grid,np.zeros(2),.02,{})
    peak=tracemalloc.get_traced_memory()[1]
   finally:tracemalloc.stop()
   self.assertLess(peak,grid.size*5,'export allocated full-sized int64 arrays')
   with (Path(d)/'grid.pgm').open('rb') as f:
    f.readline();f.readline();f.readline();f.seek(-grid.shape[1],2);self.assertEqual(f.read(3),bytes([254,0,205]))
 def test_core_default_uses_same_two_cm_cells(self):
  points=np.array([[x*.01,y*.01,.2] for x in range(4) for y in range(4)])
  implicit,origin=project(points,np.zeros(3));explicit,expected_origin=project(points,np.zeros(3),.02)
  np.testing.assert_array_equal(implicit,explicit);np.testing.assert_array_equal(origin,expected_origin)
 def test_cli_default_exports_two_cm(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);pcd=p/'input.pcd';points=np.array([[x*.01,y*.01,.2] for x in range(4) for y in range(4)],dtype='<f4')
   pcd.write_bytes(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 16\nDATA binary\n'+points.tobytes())
   subprocess.run([sys.executable,str(ROOT/'scripts/pointmap_grid.py'),'--pcd',str(pcd),'--floor','0','0','0','--output',str(p/'grid')],check=True,capture_output=True)
   self.assertEqual(yaml.safe_load((p/'grid.yaml').read_text())['resolution'],.02)
   self.assertEqual(json.loads((p/'grid.json').read_text())['resolution_m'],.02)
 def test_cold_start_launch_uses_two_cm(self):
  from launch import LaunchContext
  from launch.actions import ExecuteProcess
  from launch.utilities import perform_substitutions
  with tempfile.TemporaryDirectory() as d:
   os.environ['ARX_SINGLE_RUN']=d
   spec=importlib.util.spec_from_file_location('mapping_launch',ROOT/'single_bottom/mapping.launch.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
   commands=[[perform_substitutions(LaunchContext(),part) for part in e.cmd] for e in m.generate_launch_description().entities if isinstance(e,ExecuteProcess)]
   cmd=next(c for c in commands if any('pointmap_grid.py' in v for v in c));self.assertEqual(float(cmd[cmd.index('--resolution')+1]),.02)
 def test_current_diagnostic_extent_is_valid_at_two_cm_and_divergence_rejected(self):
  pts=np.array([[x,y,.2] for x in (-51.1,73.2) for y in (-96.7,74.7)]*4)
  grid,origin=project(pts,np.zeros(3),.02);self.assertEqual(grid.dtype,np.int8);self.assertGreater(grid.size,4_000_000)
  self.assertEqual(int((grid==100).sum()),4)
  with self.assertRaisesRegex(ValueError,'bounds exceed'):project(pts*10,np.zeros(3),.02)
if __name__=='__main__':unittest.main()
