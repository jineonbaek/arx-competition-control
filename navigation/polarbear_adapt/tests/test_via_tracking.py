import unittest
import numpy as np
from via_tracking import controller_route
class ViaPolicy(unittest.TestCase):
 def test_clip_at_exact_via_without_changing_semantic_target(self):
  path=np.array([[0.,0.,0.],[.4,.08,.1],[1.,0.,.2]])
  v=dict(id='a',pose=[.4,0.,2.],pass_radius_m=.1)
  out=controller_route(path,[v]);np.testing.assert_allclose(out[-1,:2],[.4,0.]);self.assertLess(abs(out[-1,2]),.3);np.testing.assert_allclose(path[-1],[1,0,.2])
 def test_missing_via_is_rejected_not_shortcut(self):
  with self.assertRaisesRegex(ValueError,'via_not_on_path'):controller_route([[0.,0.,0.],[1.,0.,0.]],[dict(pose=[.5,.3,0.],pass_radius_m=.1)])
 def test_no_vias_preserves_terminal_and_geometry(self):
  a=np.array([[0,0,0],[1,0,0.]])
  np.testing.assert_allclose(controller_route(a,[]),a)
