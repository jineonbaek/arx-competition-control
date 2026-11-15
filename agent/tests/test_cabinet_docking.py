import unittest
from agent.cabinet_docking import DockingPlanner,SearchBudget


def check(candidate):return dict(ik_clear=True,normal_clear=True,stopping_clear=True,coverage_complete=True,
    min_clearance_mm=40.,joint_margin_deg=20.,retreat_clear=True)


class DockingTests(unittest.TestCase):
    def test_candidates_stay_in_local_region_and_do_not_seek_lift_limit(self):
        p=DockingPlanner();r=p.propose([.5,.05,.65],current_lift_rad=5.,check=check)
        self.assertEqual(r['status'],'candidate');c=r['selected'];self.assertLessEqual(abs(c['yaw_rad']),.262)
        self.assertLessEqual(sum(v*v for v in c['base_delta_m'])**.5,.30);self.assertTrue(0<=c['lift_rad']<=18.)
        self.assertFalse(r['executable']);self.assertFalse(r['physical_clearance_verified'])
    def test_no_physical_checker_returns_proposals_not_permission(self):
        r=DockingPlanner().propose([.5,.05,.65],current_lift_rad=5.)
        self.assertEqual(r['status'],'blocked');self.assertIsNone(r['selected'])
    def test_normal_safe_but_stopping_unsafe_is_rejected(self):
        def bad(c):r=check(c);r['stopping_clear']=False;return r
        r=DockingPlanner().propose([.5,0,.65],current_lift_rad=5.,check=bad);self.assertEqual(r['status'],'blocked')
    def test_choose_clearance_before_shortest_distance(self):
        def score(c):r=check(c);r['min_clearance_mm']=80 if c['base_delta_m'][0]>.01 else 10;return r
        r=DockingPlanner().propose([.6,0,.65],current_lift_rad=5.,check=score)
        self.assertGreater(r['selected']['base_delta_m'][0],.01)
    def test_cumulative_budget_does_not_reset_when_returning_to_origin(self):
        b=SearchBudget();b.commit([.2,0],0.);b.commit([0.,0.],0.);b.commit([.1,0],0.)
        with self.assertRaises(ValueError):b.commit([-.1,0],0.)
    def test_region_yaw_and_viewpoint_budget_are_enforced(self):
        for xy,yaw in [([.4,0],0),([0,0],.4)]:
            with self.assertRaises(ValueError):SearchBudget().commit(xy,yaw)
        b=SearchBudget(max_views=2);b.commit([0,0],0);b.commit([.01,0],0)
        with self.assertRaises(ValueError):b.commit([0,0],0)
    def test_nan_and_uncertified_checker_results_are_rejected(self):
        def invalid(c):r=check(c);r['min_clearance_mm']=float('nan');return r
        self.assertEqual(DockingPlanner().propose([.5,0,.65],current_lift_rad=5.,check=invalid)['status'],'blocked')
    def test_joint_limit_retreat_and_full_sweep_failures_discard_candidates(self):
        for key,value in [('joint_margin_deg',0.),('retreat_clear',False),('normal_clear',False),('ik_clear',False)]:
            def rejected(c):r=check(c);r[key]=value;return r
            self.assertEqual(DockingPlanner().propose([.5,0,.65],current_lift_rad=5.,check=rejected)['status'],'blocked')
