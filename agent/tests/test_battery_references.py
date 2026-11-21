import unittest
from agent.battery_references import REFERENCES,reference_image
from battery_task import BatteryTask,DEFAULT_RECOGNITION_PROFILES

class ReferenceTests(unittest.TestCase):
    def test_four_real_frames_crop_to_reviewed_battery_regions(self):
        self.assertEqual(len(REFERENCES),4)
        for identity,row in REFERENCES.items():
            image=reference_image(identity)
            self.assertEqual(image.size,(row['box'][2]-row['box'][0],row['box'][3]-row['box'][1]))
        with self.assertRaises(ValueError):reference_image('../../other')

    def test_reference_ids_roundtrip_and_reject_unknown_paths(self):
        profile=DEFAULT_RECOGNITION_PROFILES['battery']
        self.assertEqual(BatteryTask._recognition_profile(profile)['reference_images'],profile['reference_images'])
        for invalid in ({'first':['../../other'],'reobserve':['d405-04']},
                        {'first':[],'reobserve':['d405-04']},
                        {'first':[{}],'reobserve':['d405-04']}):
            with self.assertRaises(ValueError):BatteryTask._recognition_profile(profile|{'reference_images':invalid})
