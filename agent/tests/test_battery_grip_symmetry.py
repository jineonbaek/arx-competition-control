import json
import unittest
from pathlib import Path

import numpy as np

from foundationpose.battery_grip import transform_reference


class BatteryGripSymmetryTests(unittest.TestCase):
    def test_upward_symmetric_bump_is_canonicalized_downward(self):
        profile=json.loads((Path(__file__).resolve().parents[2]/"web/public/cad/battery-grip.json").read_text())
        transform=np.array([[0,0,-1,0],[-1,0,0,0],[0,1,0,0],[0,0,0,1]],float)
        result=transform_reference(profile,transform,transform)
        self.assertLess(result["bump_down_error_deg"],1e-6)
        np.testing.assert_allclose(result["approach_base"],[1,0,0],atol=1e-6)


if __name__=="__main__":
    unittest.main()
