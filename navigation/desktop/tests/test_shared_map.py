import tempfile,unittest,hashlib
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from competition_map import publish,resolve_map
from mapping_direction import alignment,stationary
class SharedMap(unittest.TestCase):
 def snapshot(self,root,name,content):
  p=root/name;p.mkdir()
  for f in ['room.yaml','room.pgm','room.dual-origin.npz']:(p/f).write_bytes(content)
  (p/'SHA256SUMS').write_text(''.join(hashlib.sha256((p/f).read_bytes()).hexdigest()+'  '+f+'\n' for f in ['room.yaml','room.pgm','room.dual-origin.npz']))
  return p
 def test_publish_is_shared_and_pinned_version_is_immutable(self):
  with tempfile.TemporaryDirectory() as t:
   root=Path(t);a=self.snapshot(root,'a',b'first');publish(root,a);pinned=resolve_map(root)
   b=self.snapshot(root,'b',b'second');publish(root,b)
   self.assertEqual(pinned.read_bytes(),b'first');self.assertEqual(resolve_map(root).read_bytes(),b'second')
 def test_incomplete_or_corrupt_save_cannot_replace_latest(self):
  with tempfile.TemporaryDirectory() as t:
   root=Path(t);a=self.snapshot(root,'a',b'good');publish(root,a);old=resolve_map(root)
   b=self.snapshot(root,'b',b'bad');(b/'INCOMPLETE').touch()
   with self.assertRaises(ValueError):publish(root,b)
   (b/'INCOMPLETE').unlink();(b/'room.pgm').write_bytes(b'corrupted')
   with self.assertRaises(ValueError):publish(root,b)
   self.assertEqual(resolve_map(root),old)
 def test_failed_pointer_update_restores_legacy_directory(self):
  from unittest.mock import patch
  with tempfile.TemporaryDirectory() as t:
   root=Path(t);(root/'maps').mkdir();self.snapshot(root/'maps','比赛地图',b'original')
   new=self.snapshot(root,'new',b'new')
   with patch('competition_map.os.replace',side_effect=OSError('injected write failure')):
    with self.assertRaises(OSError):publish(root,new)
   self.assertEqual(resolve_map(root).read_bytes(),b'original')
 def test_heading_maps_current_body_to_zero_pose(self):
  import numpy as np
  rot,shift=alignment([2.,3.,1.2]);np.testing.assert_allclose(rot@np.array([2.,3.])+shift,[0,0],atol=1e-12)
  self.assertAlmostEqual(__import__('math').atan2(rot[1,0],rot[0,0])+1.2,0.)
 def test_static_window_rejects_motion_and_old_samples(self):
  samples=[(i*.1,[0.,0.,0.]) for i in range(13)]
  self.assertTrue(stationary(samples));self.assertFalse(stationary(samples[:3]))
  samples[-1]=(1.2,[.05,0.,0.]);self.assertFalse(stationary(samples))
if __name__=='__main__':unittest.main()
