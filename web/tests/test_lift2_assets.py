import hashlib
import json
import struct
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
MODEL = json.loads((ROOT / 'web/models-lift2.json').read_text())['model']
PACKAGE = ROOT / 'web/public' / MODEL['public_path']


class Lift2AssetsTests(unittest.TestCase):
    def test_installed_delivery_is_complete_and_byte_exact(self):
        robot = ET.parse(PACKAGE / MODEL['urdf']).getroot()
        meshes = {m.get('filename').removeprefix('package://lift2/') for m in robot.findall('.//mesh')}
        self.assertEqual(meshes | {MODEL['urdf']}, set(MODEL['sha256']))
        for name, digest in MODEL['sha256'].items():
            self.assertEqual(hashlib.sha256((PACKAGE / name).read_bytes()).hexdigest(), digest, name)

    def test_corrected_mesh_serves_visual_and_collision(self):
        body = ET.parse(PACKAGE / MODEL['urdf']).find("link[@name='body4']")
        for kind in ['visual','collision']:
            self.assertEqual(body.find(kind+'/geometry/mesh').get('filename'), 'package://lift2/meshes/body4.STL')
        data = (PACKAGE / 'meshes/body4.STL').read_bytes()
        self.assertEqual(struct.unpack_from('<I', data, 80)[0], 66836)
        self.assertEqual(len(data), 84 + 50 * 66836)

    def test_fixed_sensors_and_feedback_joint_names(self):
        root = ET.parse(PACKAGE / MODEL['urdf']).getroot()
        self.assertEqual(len(root.findall('link')), 27)
        joints = {j.get('name'): j for j in root.findall('joint')}
        self.assertEqual(len(joints), 26)
        self.assertEqual(joints['joint4'].get('type'), 'prismatic')
        for side, start in [('left',11),('right',21)]:
            for i in range(6):
                self.assertEqual(joints[f'{side}_joint{start+i}'].get('type'), 'revolute')
        fixed = {j.find('child').get('link'):j.find('parent').get('link') for j in joints.values() if j.get('type')=='fixed'}
        self.assertEqual(fixed, {'head_113_link':'base_link','upper_mid360_link':'head_113_link','d435i_link':'head_113_link','astra_pro_link':'head_113_link','front_hang_mount_link':'base_link','lower_mid360_link':'front_hang_mount_link'})
        for name in fixed:
            mesh = root.find(f"link[@name='{name}']/visual/geometry/mesh")
            self.assertEqual([float(x) for x in mesh.get('scale').split()], [.001]*3)

    def test_v2_fingers_preserve_mounting_transform_in_meters(self):
        settings = MODEL["gripper_v2"]
        folder = ROOT / "web/cad/gripper-v2"
        for side, start in [("left", 17), ("right", 27)]:
            for index, sign in enumerate(["positive_y", "negative_y"]):
                source = (folder / settings[sign + "_finger"]).read_bytes()
                count = struct.unpack_from("<I", source, 80)[0]
                mesh = (PACKAGE / f"meshes/{side}_link{start+index}.STL").read_bytes()
                total = struct.unpack_from("<I", mesh, 80)[0]
                # The unchanged carriage consists of 3818 triangles.
                self.assertEqual(total, count + 3818)
                shift = settings["raw_to_finger_translation_mm"][sign]
                for triangle in range(count):
                    raw = struct.unpack_from("<9f", source, 84 + triangle * 50 + 12)
                    placed = struct.unpack_from("<9f", mesh, 84 + (3818 + triangle) * 50 + 12)
                    for coordinate, (actual, value) in enumerate(zip(placed, raw)):
                        self.assertAlmostEqual(actual, (value + shift[coordinate % 3]) / 1000, places=7)

    def test_supplied_battery_has_expected_dimensions(self):
        battery = MODEL["battery"]
        data = (ROOT / battery["path"]).read_bytes()
        self.assertEqual(hashlib.sha256(data).hexdigest(), battery["sha256"])
        count = struct.unpack_from("<I", data, 80)[0]
        self.assertEqual(len(data), 84 + count * 50)
        vertices = [struct.unpack_from("<3f", data, 84 + i * 50 + 12 + j * 12)
                    for i in range(count) for j in range(3)]
        for axis, expected in enumerate([82.35, 75, 195.28]):
            self.assertAlmostEqual(max(v[axis] for v in vertices) - min(v[axis] for v in vertices),
                                   expected, places=3)
