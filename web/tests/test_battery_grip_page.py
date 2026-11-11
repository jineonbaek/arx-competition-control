import hashlib
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
PUBLIC=ROOT/"web/public"


class BatteryGripPageTests(unittest.TestCase):
    def test_reference_artifact_matches_all_supplied_cad(self):
        profile=json.loads((PUBLIC/"cad/battery-grip.json").read_text())
        for name,expected in profile["mesh_sha256"].items():
            actual=hashlib.sha256((ROOT/"web/cad/gripper-v2"/name).read_bytes()).hexdigest()
            self.assertEqual(actual,expected)
        self.assertEqual(profile["required_j6_deg"],-90)
        self.assertEqual(profile["insertion_mm"],26)
        self.assertAlmostEqual(profile["points_object_mm"]["front"][2]-profile["points_object_mm"]["tip"][2],26)
        self.assertLess(profile["hole_bounds_mm"][1][2],profile["points_object_mm"]["grip"][2])
        self.assertLess(profile["points_object_mm"]["grip"][2],profile["points_object_mm"]["front"][2])
        self.assertLess(profile["tip_alignment_error_mm"],.001)
        covered=profile["recess_coverage"]
        self.assertEqual(covered["inside_recess_samples"],covered["protrusion_vertex_samples"])

    def test_diagram_keeps_aligned_end_rails_and_distinguishes_inner_recess(self):
        text=(PUBLIC/"cad/battery-grip.svg").read_text()
        ET.fromstring(text)
        self.assertIn("两指尖齐平",text)
        self.assertIn("内侧凹槽底面",text)
        html=(PUBLIC/"foundationpose.html").read_text()
        self.assertIn('src="cad/battery-grip.svg"',html)
        self.assertIn('id="grip-geometry"',html)
        self.assertIn("不是从机械臂当前位置",html)
