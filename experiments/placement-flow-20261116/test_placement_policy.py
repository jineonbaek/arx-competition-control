import unittest
from placement_policy import PlacementPolicy, right_offset_mm, choose_alignment, replay_offline, evaluate_offline

class PlacementPolicyTest(unittest.TestCase):
    def test_requested_order_and_no_table_stage(self):
        p=PlacementPolicy()
        self.assertEqual(p.phases,("prepare","scan_plan","dual_camera_align","insert_5mm","hold"))
        self.assertEqual(p.insertion_mm,5.)
        self.assertFalse(p.requires_table)
        self.assertEqual(p.gripper_action,"retain")
    def test_signed_right_offset_has_closed_bounds(self):
        for offset in (0.,0.01,8.):
            self.assertAlmostEqual(right_offset_mm(25.,25.-offset),offset)
        for measured in (25.001,16.999):
            with self.assertRaises(ValueError):right_offset_mm(25.,measured)
    def test_d405_preferred_and_d435_fallback(self):
        a={"camera":"d405","valid":True,"axis":"vertical_offset","value":2.}
        b={"camera":"d435","valid":True,"axis":"vertical_offset","value":1.}
        self.assertIs(choose_alignment(a,b),a)
        self.assertIs(choose_alignment(None,b),b)
        self.assertIsNone(choose_alignment(None,None))
    def test_d405_offset_is_not_a_tilt_measurement(self):
        a={"camera":"d405","valid":True,"axis":"vertical_offset","value":2.}
        self.assertIsNone(choose_alignment(a,None,axis="tilt"))
    def test_nonfinite_inputs_are_rejected(self):
        for v in (float("nan"),float("inf"),-float("inf")):
            with self.assertRaises(ValueError):right_offset_mm(25.,v)
        self.assertIsNone(choose_alignment({"camera":"d405","valid":True,"axis":"tilt","value":float("nan")},None,axis="tilt"))

    def test_offline_replay_uses_the_requested_order(self):
        seen=[]
        callbacks={name:(lambda name=name: seen.append(name)) for name in PlacementPolicy().phases}
        replay_offline(callbacks)
        self.assertEqual(seen,list(PlacementPolicy().phases))
        with self.assertRaises(ValueError): replay_offline({})

    def test_reusable_offline_evaluation_selects_sensors_and_reports_advisories(self):
        scene={"left_reference_uv_mm":25.,"measured_uv_mm":21.,"d405":{"camera":"d405","valid":True,"axis":"vertical_offset","value":2.},"d435":{"camera":"d435","valid":True,"axis":"tilt","value":1.2},"grip":"unknown"}
        result=evaluate_offline(scene)
        self.assertEqual(result["right_offset_mm"],4.)
        self.assertEqual(result["vertical"]["camera"],"d405")
        self.assertEqual(result["tilt"]["camera"],"d435")
        self.assertFalse(result["executable"])
        self.assertIn("grip_unknown",result["advisories"])
    def test_offline_evaluation_does_not_invent_missing_measurements(self):
        result=evaluate_offline({"left_reference_uv_mm":25.,"measured_uv_mm":25.,"grip":"stable"})
        self.assertIsNone(result["vertical"])
        self.assertIsNone(result["tilt"])
        self.assertIn("tilt_unavailable",result["advisories"])
        self.assertFalse(result["executable"])
if __name__=="__main__":unittest.main()
