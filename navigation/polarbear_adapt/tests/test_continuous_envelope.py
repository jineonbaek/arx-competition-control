"""Continuous collision coverage without travel-sized isotropic overpadding."""
import math
import os
import importlib.util
import unittest
import numpy as np
from path_audit import AuditGrid


class ContinuousEnvelope(unittest.TestCase):
    def setUp(self):
        self.poly = np.array([[-.19, -.19], [.19, -.19], [.19, .19], [-.19, .19]])

    def grid(self, point, value=100, res=.05, poly=None):
        side = round(4. / res)
        cells = np.zeros((side, side), dtype=int)
        origin = np.array([-2., -2.])
        i, j = np.floor((np.array(point)-origin)/res).astype(int)
        cells[j, i] = value
        return AuditGrid(cells, origin, res, self.poly if poly is None else poly)

    def test_translation_away_from_wall_preserves_real_one_centimetre_gap(self):
        g = self.grid([-.225, -.025])
        a, b = np.array([0., 0., 0.]), np.array([.10, 0., 0.])
        self.assertIsNone(g.inspect(a)['first_cell'])
        self.assertIsNone(g.inspect(b)['first_cell'])
        self.assertIsNone(g.along(np.array([a, b]))['first'])

    def test_parallel_motion_does_not_consume_unrelated_side_clearance(self):
        g = self.grid([-.225, -.025])
        self.assertIsNone(g.along(np.array([[0., 0., 0.], [0., .15, 0.]]))['first'])

    def test_thin_collision_strictly_between_clear_endpoints_is_rejected(self):
        tiny = self.poly * .01
        g = self.grid([.025, .025], poly=tiny)
        a, b = np.array([-.08, .025, 0.]), np.array([.08, .025, 0.])
        self.assertIsNone(g.inspect(a)['first_cell'])
        self.assertIsNone(g.inspect(b)['first_cell'])
        self.assertIsNotNone(g.along(np.array([a, b]))['first'])

    def test_rotating_corner_arc_between_clear_endpoints_is_rejected(self):
        finger = np.array([[-.005, -.005], [.6, -.005], [.6, .005], [-.005, .005]])
        g = self.grid([.595, .005], res=.01, poly=finger)
        a, b = np.array([0., 0., -.4]), np.array([0., 0., .4])
        self.assertIsNone(g.inspect(a)['first_cell'])
        self.assertIsNone(g.inspect(b)['first_cell'])
        self.assertIsNotNone(g.along(np.array([a, b]))['first'])

    def test_short_yaw_wrap_covers_the_protruding_corner(self):
        finger = np.array([[-.005, -.005], [.6, -.005], [.6, .005], [-.005, .005]])
        cells = np.zeros((400, 400), dtype=int)
        cells[200, 140] = 100
        g = AuditGrid(cells, [-2., -2.], .01, finger)
        a, b = np.array([0., 0., math.pi-.4]), np.array([0., 0., -math.pi+.4])
        self.assertIsNone(g.inspect(a)['first_cell'])
        self.assertIsNone(g.inspect(b)['first_cell'])
        self.assertIsNotNone(g.along(np.array([a, b]))['first'])

    def test_single_interval_rotational_extremum_requires_curvature_allowance(self):
        finger = np.array([[-.000001, -.000001], [.6, -.000001],
                           [.6, .000001], [-.000001, .000001]])
        cells = np.zeros((80, 80), dtype=int)
        cells[40, 40] = 100
        # The cell starts at x=.59995. Both endpoint polygons and their
        # convex hull end before x=.59989, but the true middle tip is x=.6.
        g = AuditGrid(cells, [-1.40005, -2.], .05, finger)
        a, b = np.array([0., 0., -.02]), np.array([0., 0., .02])
        self.assertIsNone(g.inspect(a)['first_cell'])
        self.assertIsNone(g.inspect(b)['first_cell'])
        self.assertIsNotNone(g.along(np.array([a, b]))['first'])

    def test_final_endpoint_and_single_pose_collision_are_rejected(self):
        tiny = self.poly * .01
        g = self.grid([.225, .025], poly=tiny)
        a, b = np.array([-.10, .025, 0.]), np.array([.199, .025, 0.])
        self.assertIsNone(g.inspect(a)['first_cell'])
        self.assertIsNotNone(g.inspect(b)['first_cell'])
        self.assertIsNotNone(g.along(np.array([a, b]))['first'])
        self.assertIsNotNone(g.along(np.array([b]))['first'])

    def test_unknown_and_current_overlap_remain_blocked_without_mutation(self):
        for value in (-1, 100):
            g = self.grid([.025, .025], value=value)
            before = g.grid.copy()
            self.assertIsNotNone(g.along(np.array([[0., 0., 0.], [.1, 0., .2]]))['first'])
            np.testing.assert_array_equal(g.grid, before)
            np.testing.assert_array_equal(g.poly, self.poly)

    @unittest.skipUnless(os.getenv('ARX_AUDIT_BASELINE'), 'baseline supplied by isolated review')
    def test_inspect_optimization_preserves_existing_cell_counts_and_first_hit(self):
        spec = importlib.util.spec_from_file_location('audit_baseline', os.environ['ARX_AUDIT_BASELINE'])
        baseline = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(baseline)
        rng = np.random.default_rng(321)
        cells = rng.choice([-1, 0, 0, 99, 100], (40, 40))
        old = baseline.AuditGrid(cells.copy(), [-1., -1.], .05, self.poly)
        new = AuditGrid(cells.copy(), [-1., -1.], .05, self.poly)
        for _ in range(80):
            pose = np.r_[rng.uniform(-1.1, 1.1, 2), rng.uniform(-math.pi, math.pi)]
            margin = rng.uniform(0., .04)
            self.assertEqual(new.inspect(pose, margin), old.inspect(pose, margin))

    def test_seeded_actual_corner_contacts_along_rigid_motion_are_never_missed(self):
        rng = np.random.default_rng(812)
        for _ in range(80):
            a = np.r_[rng.uniform(-.4, .4, 2), rng.uniform(-math.pi, math.pi)]
            b = a + np.r_[rng.uniform(-.4, .4, 2), rng.uniform(-2.8, 2.8)]
            t = rng.uniform(.02, .98)
            p = a + t * (b-a)
            v = self.poly[rng.integers(len(self.poly))]
            c, s = math.cos(p[2]), math.sin(p[2])
            touched = p[:2] + [c*v[0]-s*v[1], s*v[0]+c*v[1]]
            g = self.grid(touched)
            self.assertIsNotNone(g.along(np.array([a, b]))['first'])

    @unittest.skipUnless(os.getenv('ARX_BOUNDARY_CASE'), 'live case provided by commissioning run')
    def test_real_current_overlap_is_not_made_free_by_the_new_envelope(self):
        d = np.load(os.environ['ARX_BOUNDARY_CASE'])
        g = AuditGrid(d['grid'], d['origin'], float(d['resolution']), d['footprint'])
        self.assertIsNotNone(g.inspect(d['pose'])['first_cell'])
        self.assertIsNotNone(g.along(np.array([d['pose'], d['goal']]))['first'])


if __name__ == '__main__':
    unittest.main()
