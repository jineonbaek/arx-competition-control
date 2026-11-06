import unittest
from benchmark import summarize


class BenchmarkStatisticsTests(unittest.TestCase):
    def test_summary_uses_median_and_interpolated_p95(self):
        result = summarize([30., 10., 20.])
        self.assertEqual(result["count"], 3)
        self.assertEqual(result["median_ms"], 20.)
        self.assertEqual(result["p95_ms"], 29.)
        self.assertEqual(result["mean_ms"], 20.)

    def test_single_sample_has_no_invented_distribution(self):
        result = summarize([12.])
        self.assertEqual(result["median_ms"], 12.)
        self.assertEqual(result["p95_ms"], 12.)

    def test_invalid_samples_are_rejected(self):
        for values in [[], [0.], [-1.], [float("nan")], [float("inf")]]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                summarize(values)


if __name__ == "__main__":
    unittest.main()
