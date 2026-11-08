import json
import importlib.util
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("prepare_lift2_preview", ROOT / "scripts/prepare_lift2_preview.py")

class Lift2AssetsTests(unittest.TestCase):
    def loader(self):
        self.assertTrue(Path(SPEC.origin).is_file(), "Missing pinned LIFT2 asset preparation script")
        module = importlib.util.module_from_spec(SPEC)
        SPEC.loader.exec_module(module)
        return module

    def test_git_blob_integrity_rejects_corruption(self):
        m = self.loader()
        m.check_blob(b"hello\n", "ce013625030ba8dba906f756967f9e9ca394464a")
        with self.assertRaises(ValueError):
            m.check_blob(b"changed", "ce013625030ba8dba906f756967f9e9ca394464a")

    def test_asset_record_paths_work_across_windows_and_linux(self):
        record = json.loads((ROOT / "web/public/models/lift2/asset-integrity.json").read_text())
        for name in record["sha256"]:
            self.assertNotIn(chr(92), name)
            self.assertTrue((ROOT / "web/public" / name).is_file(), name)

    def test_official_model_contains_both_arms_and_lift(self):
        self.loader()
        public = ROOT / "web/public"
        model = ET.parse(public / "models/lift2/urdf/lift2.urdf").getroot()
        self.assertEqual(len(model.findall("link")), 21)
        joints = {j.get("name"): j for j in model.findall("joint")}
        self.assertEqual(joints["joint4"].get("type"), "prismatic")
        self.assertAlmostEqual(float(joints["joint4"].find("limit").get("upper")), .46)
        for side, first in (("left", 11), ("right", 21)):
            for n in range(first, first + 6):
                self.assertIn(f"{side}_joint{n}", joints)
        for mesh in model.findall(".//mesh"):
            relative = mesh.get("filename").removeprefix("package://lift2/")
            self.assertTrue((public / "models/lift2" / relative).is_file(), relative)

if __name__ == "__main__":
    unittest.main()
