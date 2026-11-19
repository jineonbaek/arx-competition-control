"""Supervised endpoint proposal boundaries; no transport or SDK objects."""
import unittest
from agent.lift_endpoint import next_reference,settled_height


class EndpointTests(unittest.TestCase):
    def test_measured_ascent_error_proposes_bounded_positive_reference(self):
        r=next_reference(.95,.95,.7040128707885742,corrections=0)
        self.assertEqual(r['status'],'correct')
        self.assertAlmostEqual(r['reference'],1.1959871292114257)

    def test_measured_return_error_proposes_bounded_negative_reference(self):
        r=next_reference(.45,.45,.7021158321483715,corrections=0)
        self.assertEqual(r['status'],'correct')
        self.assertAlmostEqual(r['reference'],.1978841678516285)

    def test_reference_and_total_correction_limits_stop_windup(self):
        r=next_reference(.45,.15,.70,corrections=1)
        self.assertEqual(r['status'],'reference_limit')
        self.assertIsNone(r['reference'])
        r=next_reference(.95,1.25,.7,corrections=3)
        self.assertEqual(r['status'],'correction_limit')
        self.assertIsNone(r['reference'])
        for target,reference,measured in [(0.,0.,.25),(18.,18.,17.75)]:
            self.assertEqual(next_reference(target,reference,measured,corrections=0)['status'],
                             'reference_limit')

    def test_single_step_and_operator_bounds_are_enforced(self):
        r=next_reference(.95,.95,.3,corrections=0,reference_bounds=(.15,1.25))
        self.assertEqual(r['reference'],1.25)
        r=next_reference(.45,.45,1.,corrections=0,reference_bounds=(.15,1.25))
        self.assertAlmostEqual(r['reference'],.15)

    def test_actual_feedback_inside_tolerance_finishes_without_another_command(self):
        r=next_reference(.45,.2,.451,corrections=3)
        self.assertEqual(r['status'],'arrived');self.assertIsNone(r['reference'])

    def test_nonfinite_invalid_count_or_unapproved_reference_is_rejected(self):
        cases=[dict(target=float('nan')),dict(measured=float('inf')),dict(reference=19.),
               dict(corrections=True),dict(corrections=4),dict(reference_bounds=(1.,0.)),
               dict(reference_bounds=(0.,)),dict(reference_bounds=(0.,1.,2.))]
        for changes in cases:
            args=dict(target=.45,reference=.45,measured=.7,corrections=0)
            args.update(changes)
            with self.subTest(changes=changes),self.assertRaises(ValueError):next_reference(**args)

    def test_stable_fresh_distinct_packets_produce_measured_height(self):
        samples=[dict(height=.704,received=10+i*.1,now=10+i*.1+.01) for i in range(21)]
        self.assertAlmostEqual(settled_height(samples,target=.95),.704)

    def test_cached_stale_moving_or_regressing_samples_never_establish_settling(self):
        cases=[
            [dict(height=.704,received=10.,now=10.+i*.01) for i in range(21)],
            [dict(height=.704,received=10+i*.1,now=11+i*.1) for i in range(21)],
            [dict(height=.704+i*.001,received=10+i*.1,now=10+i*.1+.01) for i in range(21)],
            [dict(height=.704,received=12-i*.1,now=12.1) for i in range(21)],
            [dict(height=.704,received=10+i,now=10+i+.01) for i in range(3)],
        ]
        for samples in cases:
            with self.subTest(samples=samples[:2]),self.assertRaises(ValueError):settled_height(samples,target=.95)

    def test_a_jump_just_outside_window_cannot_borrow_older_stable_time(self):
        stamps=[0.,.28,.52,.77,1.02,1.27,1.51]
        samples=[dict(height=.55 if i==0 else .70,received=t,now=t+.01)
                 for i,t in enumerate(stamps)]
        with self.assertRaises(ValueError):settled_height(samples,target=.95)

    def test_window_mean_cannot_hide_an_endpoint_outside_tolerance(self):
        samples=[dict(height=.469 if i<15 else .473,received=10+i*.1,now=10+i*.1+.01)
                 for i in range(16)]
        measured=settled_height(samples,target=.45)
        result=next_reference(.45,.45,measured,corrections=0)
        self.assertNotEqual(result['status'],'arrived')


if __name__=='__main__':unittest.main()
