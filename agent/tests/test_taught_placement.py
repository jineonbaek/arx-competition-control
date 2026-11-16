import json
import tempfile
import unittest
from pathlib import Path


class Clock:
    def __init__(self): self.mono=10.; self.wall=100.
    def monotonic(self): return self.mono
    def time(self): return self.wall


class TaughtPlacementTests(unittest.TestCase):
    def test_timeline_records_success_failure_and_total_with_monotonic_time(self):
        from agent.taught_placement import PhaseTimeline
        c=Clock(); t=PhaseTimeline(c.monotonic,c.time)
        t.start('observe');t.plan(2.);c.mono+=1.25;c.wall-=50.;t.finish('ok')
        t.start('trim');c.mono+=.5;t.finish('failed','no edge')
        c.mono+=.25;t.complete()
        self.assertEqual([x['phase'] for x in t.events],['observe','trim'])
        self.assertAlmostEqual(t.events[0]['duration_s'],1.25)
        self.assertEqual(t.events[1]['status'],'failed')
        self.assertEqual(t.events[1]['reason'],'no edge')
        self.assertEqual(t.events[0]['planned_duration_s'],2.)
        self.assertAlmostEqual(t.events[0]['cumulative_duration_s'],1.25)
        self.assertAlmostEqual(t.events[1]['cumulative_duration_s'],1.75)
        self.assertAlmostEqual(t.snapshot()['total_duration_s'],2.)

    def test_timeline_rejects_invalid_or_repeated_plan(self):
        from agent.taught_placement import PhaseTimeline
        t=PhaseTimeline()
        with self.assertRaises(ValueError):t.plan(1.)
        t.start('move')
        with self.assertRaises(ValueError):t.plan(0.)
        t.plan(3.)
        self.assertEqual(t.snapshot()['active_phase']['planned_duration_s'],3.)
        with self.assertRaises(ValueError):t.plan(4.)

    def test_outer_alignment_error_uses_current_window_left_slot(self):
        from agent.taught_placement import outer_alignment_error
        fit={'footprint_uv_mm':[[2.,-38.],[84.,-38.],[84.,38.],[2.,38.]],'leading_standoff_mm':60.}
        window={'measured_size_mm':[172.,80.]};slot={'size_mm':[174.,82.]}
        result=outer_alignment_error(fit,window,slot)
        self.assertEqual(result['target_uv_mm'],[43.,0.])
        self.assertEqual(result['center_uv_mm'],[43.,0.])
        self.assertEqual(result['residual_uv_mm'],[0.,0.])
        self.assertTrue(result['converged'])
        moved=outer_alignment_error(fit,{'measured_size_mm':[160.,80.]},slot)
        self.assertEqual(moved['target_uv_mm'],[40.,0.])
        self.assertEqual(moved['residual_uv_mm'],[-3.,0.])

    def test_outer_alignment_requires_finite_current_observation(self):
        from agent.taught_placement import outer_alignment_error
        with self.assertRaises(ValueError):outer_alignment_error(
            {'footprint_uv_mm':[[0.,0.]],'leading_standoff_mm':float('nan')},
            {'measured_size_mm':[172.,80.]},{'size_mm':[174.,82.]})

    def test_reverse_uses_only_reached_references(self):
        from agent.taught_placement import reverse_reached_route
        reached=[[0]*6,[1]*6,[2]*6]
        self.assertEqual(reverse_reached_route(reached),[[2]*6,[1]*6,[0]*6])
        with self.assertRaises(ValueError): reverse_reached_route([[0]*5])

    def test_versioned_route_loads_three_rad_and_success_depth(self):
        from agent.taught_placement import load_taught_route
        root=Path(__file__).resolve().parents[2]
        route=load_taught_route(root/'config/taught-placement.json')
        self.assertEqual(route.version,1)
        self.assertEqual(route.lift_target_rad,3.)
        self.assertEqual(route.lift_tolerance_rad,.35)
        self.assertAlmostEqual(route.full_depth_mm,145.5)
        self.assertEqual(route.d405_step_limit_mm,5.)
        self.assertEqual(route.d405_total_limit_mm,15.)


if __name__=='__main__': unittest.main()
