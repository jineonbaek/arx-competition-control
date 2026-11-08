import sys
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
import handeye_validation as v
from handeye_steps import Steps


class ValidationTests(unittest.TestCase):
    def test_chain_and_camera_identity(self):
        candidate=dict(parent_frame="right_link26",child_frame="D405 color optical frame",
                       camera={"serial":"test"},T_right_link26_from_D405_color=[
                           [0,-1,0,.1],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
        g=np.eye(4);g[0,3]=.2
        c=np.eye(4);c[0,3]=.05
        sample=dict(T_body4_from_right_link26=g,T_camera_from_marker=c,color_intrinsics={"serial":"test"})
        np.testing.assert_allclose(v.marker_center(sample,candidate),[300,50,0])
        sample["color_intrinsics"]["serial"]="other"
        with self.assertRaises(ValueError):v.marker_center(sample,candidate)

    def test_missing_axes_are_not_treated_as_zero(self):
        for value in ([1,2], [True,2,3], [0,0,float("nan")], "123"):
            with self.assertRaises(ValueError):v.validate_reference(value)
        self.assertIsNone(v.validate_reference(None))
        self.assertEqual(v.validate_reference([1,None,3]),[1,None,3])
        self.assertIsNone(v.validate_reference([None,None,None]))
        np.testing.assert_allclose(v.validate_reference([0,-20,10]),[0,-20,10])

    def test_validation_does_not_replace_session_or_submit_commands(self):
        controller=Mock();steps=Steps(controller,Mock())
        steps.model=Mock();steps.session="completed";steps.cursor=13
        steps.pending={"point":1}
        with self.assertRaises(ValueError):steps.action("viewer",{"op":"verify_point"})
        self.assertEqual(steps.pending,{"point":1})
        self.assertEqual(steps.session,"completed")
        controller.submit.assert_not_called()

    def test_partial_survey_saves_signed_axis_error_without_claiming_3d_accuracy(self):
        candidate=json.loads(v.CANDIDATE.read_text())
        q=np.zeros(6)
        def snapshot():
            return {"arms":{"right":{"timestamp":time.time()}}},q,0
        def frames(spec,start,arm,model,history,**kwargs):
            self.assertFalse(kwargs["color_only"])
            return dict(color_intrinsics=candidate["camera"]["intrinsics"],T_body4_from_right_link26=np.eye(4),
                        T_camera_from_marker=np.eye(4),image_time=arm["timestamp"],camera_session="test"),b""
        model=Mock()
        joint=np.eye(4);joint[0,3]=.05
        model.parts=[(joint,)]
        with tempfile.TemporaryDirectory() as tmp, patch.object(v,"CHECKS",Path(tmp)), patch.object(v,"observe",side_effect=frames):
            result=v.check_point(snapshot,model,[10,None,None])
            self.assertEqual(result["delta_mm"][1:],[None,None])
            self.assertIsNone(result["error_mm"])
            expected=1000*candidate["T_right_link26_from_D405_color"][0][3]-50-10
            self.assertAlmostEqual(expected,result["delta_mm"][0],places=3)
            self.assertEqual(json.loads((Path(tmp)/(result["check_id"]+".json")).read_text()), result)
