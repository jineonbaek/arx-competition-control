"""Small functional checks of the real CUDA operators and compatibility change."""
from pathlib import Path
import sys
import unittest
import numpy as np
import torch

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT / "upstream/graspnet-baseline" / name)
               for name in ("pointnet2", "knn", "dataset", "utils")]


class InstalledGraspNetTests(unittest.TestCase):
    def test_gpu_is_available_and_supported(self):
        self.assertTrue(torch.cuda.is_available())
        self.assertIn("sm_120", torch.cuda.get_arch_list())

    def test_farthest_point_sampling_runs_on_gpu(self):
        from pointnet2_utils import furthest_point_sample
        points = torch.zeros((1, 8, 3), device="cuda")
        points[0, :, 0] = torch.arange(8, device="cuda")
        result = furthest_point_sample(points, 4)
        torch.cuda.synchronize()
        self.assertEqual(result.unique().numel(), 4)
        self.assertEqual(result[0, :2].tolist(), [0, 7])

    def test_knn_matches_known_neighbors(self):
        from knn_modules import knn
        points = torch.zeros((1, 3, 8), device="cuda")
        points[0, 0] = torch.arange(8, device="cuda")
        result = knn(points, points[:, :, [0, 7]].contiguous(), k=1)
        torch.cuda.synchronize()
        self.assertEqual(result.tolist(), [[[1, 8]]])  # Upstream returns 1-based indices.

    def test_legacy_collation_still_batches_numpy_values(self):
        from graspnet_dataset import collate_fn
        value = collate_fn([{"point": np.array([1., 2.])}, {"point": np.array([3., 4.])}])
        self.assertEqual(value["point"].tolist(), [[1., 2.], [3., 4.]])

    def test_checkpoint_loads_without_unrestricted_pickle(self):
        checkpoint = torch.load(ROOT / "models/checkpoint-rs.tar",
                                map_location="cpu", weights_only=True)
        self.assertIn("model_state_dict", checkpoint)
        self.assertEqual(checkpoint["epoch"], 18)


if __name__ == "__main__":
    unittest.main()
