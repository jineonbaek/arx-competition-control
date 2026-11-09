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

    def test_static_two_view_geometry(self):
        from agent.competition_scene import triangulate, MAIN
        if not (MAIN/"vision/vendor-python/pyrealsense2").is_dir():
            self.skipTest("ROG RealSense runtime is required")
        import json
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            store=Mock()
            store.image_path.side_effect=lambda ident,cam:root/ident/(cam+".jpg")
            for ident,x in (("a",0.),("b",.2)):
                d=root/ident
                d.mkdir()
                T=np.eye(4);T[0,3]=x
                k=dict(width=640,height=480,cx=320.,cy=240.,fx=500.,fy=500.,
                       model="distortion.none",coeffs=[0.]*5)
                np.savez(d/"scene.npz",body4_from_color=T,xyz=np.zeros((480,640,3)),
                         intrinsics=json.dumps(k))
            result=triangulate(store,["a","b"],[[240,370],[240,270]])
            np.testing.assert_allclose(result["body4_m"],[.1,0.,1.],atol=1e-6)
            with self.assertRaises(AgentError):
                triangulate(store,["a","b"],[[240,320],[240,320]])
