import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ADAPT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ADAPT))
from chassis_solid import ChassisSolid
from clear_votes import SIZE, TransientEvidence


class RigidBodyClearTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model = ADAPT.parent / 'models/lift2s_bottom_upright_20261105/base_interior.npz'
        cls.solid = ChassisSolid(model)
        with np.load(model) as grid:
            cells = np.argwhere(grid['solid'])
            cls.inside = (cells[len(cells) // 2] + grid['lower']) * grid['pitch']

    def test_only_certified_old_height_removed(self):
        empty = np.empty((0, 3))
        for r, t in [(np.eye(3), np.zeros(3)),
                     (Rotation.from_euler('xyz', [.04, -.03, .83]).as_matrix(),
                      np.array([1., -2., .017]))]:
            with self.subTest(rotated=bool(t.any())):
                local = np.array([self.inside, [*self.inside[:2], .7],
                                  [.375, .253, .45], [.375, -.253, .45], [.7, 0., .08]])
                hits = local @ r.T + t
                world = TransientEvidence(self.solid)
                keys = [tuple(k) for k in np.floor(hits / SIZE).astype(int)]
                world.update('bottom', 100, t, empty, hits)
                world.update('head', 95, t, empty, empty, (r, t))
                self.assertIn(keys[0], world.cells)
                world.update('bottom', 110, t, empty, hits[:1], (r, t))
                self.assertEqual(world.solid_removed, 0)
                world.update('head', 120, t, empty, empty, (r, t))
                self.assertEqual(set(world.cells), set(keys[1:]))
                self.assertEqual(world.solid_removed, 1)
                if not t.any():
                    self.assertEqual(keys[0][:2], keys[1][:2])
                    projected = world.projection()
                    free = np.floor(projected[projected[:, 2] < 0, :2] / SIZE[:2]).astype(int)
                    self.assertNotIn(keys[0][:2], map(tuple, free))

    def test_partial_overlap_is_retained(self):
        high = self.inside.copy()
        high[2] = self.solid.deck
        contained = self.solid.contains_bounds(self.inside[None], high[None],
                                               np.eye(3), np.zeros(3))
        self.assertFalse(contained[0])


if __name__ == '__main__':
    unittest.main()
