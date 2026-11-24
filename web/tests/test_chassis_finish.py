"""Contest completion is measured gripper opening, before cleanup motion."""
import time
import unittest
from unittest.mock import patch
import test_chassis_placement as placement


class FinishOnRelease(unittest.TestCase):
    def test_finish_is_recorded_before_retreat_even_if_cleanup_fails(self):
        for fail_cleanup in (False, True):
            with self.subTest(fail_cleanup=fail_cleanup):
                finalizer, context, events = placement.FinalizeTests().fixture()
                saved = {}
                finalizer.backend.save = lambda **data: saved.update(data)
                before = time.time()
                def retreat(ctx, run, goal, guard):
                    guard()
                    self.assertTrue(run['released'])
                    self.assertGreaterEqual(run['competition_finished_at'], before)
                    self.assertEqual(saved['competition_finished_at'], run['competition_finished_at'])
                    self.assertEqual(saved['competition_finish_basis'], 'gripper_open_feedback')
                    events.append('retreat')
                    if fail_cleanup:
                        raise ValueError('return unavailable')
                    return dict(status='completed')
                finalizer._retreat = retreat
                finalizer.start(context)
                finalizer.runs[context['request_id']]['thread'].join(2.)
                result = finalizer.inspect(context)
                self.assertEqual(result['status'], 'failed' if fail_cleanup else 'completed', result)
                self.assertIn('competition_finished_at', result)
                self.assertLess(events.index('release'), events.index('retreat'))

    def test_unconfirmed_gripper_opening_does_not_finish_or_retreat(self):
        for accepted in (False, True):
            with self.subTest(command_accepted=accepted):
                finalizer, context, events = placement.FinalizeTests().fixture()
                # Acknowledgement alone must not count as an open gripper.
                finalizer.ctrl.submit = lambda *args, **kw: dict(ok=accepted)
                with patch('chassis_placement.time.monotonic', side_effect=iter(range(20))):
                    finalizer.start(context)
                    finalizer.runs[context['request_id']]['thread'].join(2.)
                result = finalizer.inspect(context)
                self.assertEqual(result['status'], 'failed')
                self.assertNotIn('competition_finished_at', result)
                self.assertNotIn('retreat', events)


if __name__ == '__main__':
    unittest.main()
