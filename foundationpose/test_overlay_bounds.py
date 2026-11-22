"""Bad display projections must not interrupt otherwise valid metric evidence."""
import json
from pathlib import Path
import unittest
import os
import subprocess
import sys
import cv2
import numpy as np

if __package__:
    from .battery_grip import draw_reference, transform_reference
    from .overlay import draw_line
else:
    from battery_grip import draw_reference, transform_reference
    from overlay import draw_line


class OverlayBoundsTests(unittest.TestCase):
    def profile(self):
        return json.loads((Path(__file__).resolve().parents[1]/'web/public/cad/battery-grip.json').read_text())

    def test_reference_outside_int32_does_not_crash_or_change_metric_geometry(self):
        profile = self.profile()
        image = np.zeros((48, 64, 3), np.uint8)
        project = lambda points: np.tile([2.**40, 24.], (len(points), 1))
        result = draw_reference(profile, np.eye(4), np.eye(4), project, image)
        expected = transform_reference(profile, np.eye(4), np.eye(4))
        for key, value in expected.items():
            self.assertEqual(result[key], value)
        self.assertFalse(image.any())
        json.dumps(result, allow_nan=False)

    def test_nonfinite_display_projection_is_omitted_from_drawing_and_json(self):
        image = np.zeros((48, 64, 3), np.uint8)
        project = lambda points: np.tile([np.nan, np.inf], (len(points), 1))
        result = draw_reference(self.profile(), np.eye(4), np.eye(4), project, image)
        self.assertFalse(image.any())
        self.assertTrue(all(v is None for v in result['pixels'].values()))
        json.dumps(result, allow_nan=False)

    def test_huge_crossing_segment_is_clipped_before_opencv(self):
        image = np.zeros((48, 64, 3), np.uint8)
        ends = draw_line(image, [-2.**40, 24.], [2.**40, 24.], (0, 255, 0), 2)
        self.assertEqual(ends, [[0, 24], [63, 24]])
        self.assertTrue(np.all(image[24, :, 1] == 255))

    def test_visible_segment_retains_original_raster(self):
        original = np.zeros((48, 64, 3), np.uint8)
        image = original.copy()
        cv2.line(original, (5, 8), (40, 30), (12, 100, 240), 2, cv2.LINE_AA)
        draw_line(image, [5, 8], [40, 30], (12, 100, 240), 2)
        np.testing.assert_array_equal(image, original)

    def test_standalone_battery_grip_entry_without_repository_pythonpath(self):
        code = '''import json, numpy as np
from pathlib import Path
from battery_grip import draw_reference
p=json.loads(Path('../web/public/cad/battery-grip.json').read_text())
image=np.zeros((48,64,3),np.uint8)
draw_reference(p,np.eye(4),np.eye(4),lambda points:np.tile([20.,20.],(len(points),1)),image)
assert image.any()
'''
        env = dict(os.environ)
        env.pop('PYTHONPATH', None)
        result = subprocess.run([sys.executable, '-c', code], env=env,
            cwd=Path(__file__).resolve().parent, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
