import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock
import numpy as np
from agent.competition_scene import pixels
from agent.contracts import AgentError

class SurfaceTest(unittest.TestCase):
    def test_row_column_depth_and_transform(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = Mock()
            store.image_path.return_value = Path(tmp)/"d405.jpg"
            xyz = np.zeros((3,5,3))
            xyz[1,4] = [0.2,0.3,0.4]
            T = np.eye(4)
            T[0,3] = 1.
            np.savez(Path(tmp)/"scene.npz",xyz=xyz,body4_from_color=T)
            result=pixels(store,"obs",[[1,4],[0,0]])
            np.testing.assert_allclose(result["points"][0]["body4_m"],[1.2,.3,.4])
            self.assertIsNone(result["points"][1]["body4_m"])
            with self.assertRaises(AgentError):
                pixels(store,"obs",[[4,1]])
