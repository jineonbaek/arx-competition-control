import unittest
import live_preview

class PreviewTests(unittest.TestCase):
    def test_pairing(self):
        self.assertTrue(live_preview.paired(10.,10.1))
        self.assertFalse(live_preview.paired(10.,10.81))
        self.assertFalse(live_preview.paired(float('nan'),10.))
    def test_unauthorized_operations_have_no_route(self):
        self.assertEqual(live_preview.MUTATING_ROUTES,('/api/capture',))
    def test_capture_is_always_non_executable(self):
        result=live_preview.base_report()
        self.assertIs(result['executable'],False)
        self.assertIs(result['robot_motion'],False)

if __name__=='__main__': unittest.main()
