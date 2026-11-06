"""Behavior tests for competition orchestration; all physical backends are fixtures."""
import importlib
import json
from pathlib import Path
import tempfile
import time
import unittest


class SystemTests(unittest.TestCase):
    def setUp(self):
        try:
            self.storage = importlib.import_module("agent.storage")
            self.tools_module = importlib.import_module("agent.tools")
            self.config_module = importlib.import_module("agent.config")
        except ImportError as error:
            self.fail(f"System framework not implemented: {error}")
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = self.storage.RunStore.create(Path(self.temp.name))
        self.addCleanup(self.store.close)
        self.config = self.config_module.Config(motion_enabled=True)
        self.backend = FixtureBackend()
        self.observations = FixtureObservations(self.store)
        self.tools = self.tools_module.ToolRegistry(self.config, self.store,
                                                  self.observations, self.backend)
        self.serial = 0

    def call(self, name, **arguments):
        self.serial += 1
        return self.tools.execute(name, arguments, f"call_{self.serial}")

    def prepare_pick(self):
        self.assertTrue(self.call("set_task", target_color="red", destination="red_slot")["ok"])
        obs = self.call("observe_scene", cameras=["d435"], view="raw")["data"]["observation_id"]
        self.assertTrue(self.call("select_target", observation_id=obs, camera="d435",
                                  bbox_xyxy=[1, 1, 10, 10], color="red")["ok"])
        self.assertTrue(self.call("generate_grasp_candidates", observation_id=obs)["ok"])
        self.assertTrue(self.call("plan_pick", candidate_id="grasp_1", arm="left")["ok"])
        return obs

    def test_unavailable_domains_are_explicit_and_never_advance_task(self):
        self.tools.backends = self.tools_module.BackendRouter({})
        self.call("set_task", target_color="red", destination="red_slot")
        result = self.call("navigate_to_area")
        self.assertFalse(result["ok"])
        self.assertEqual(self.store.state["mission"]["stage"], "TASK_ACCEPTED")
        status = self.call("get_system_status")["data"]
        self.assertFalse(status["capabilities"]["navigate_to_area"])

    def test_strict_validation_precedes_backend_or_state_changes(self):
        for name, arguments in [("execute_pick", {"plan_id": "p", "url": "http://evil"}),
                                ("shell", {"command": "anything"}),
                                ("observe_scene", {"cameras": ["unknown"]}),
                                ("set_task", {"target_color": "red", "destination": "x", "extra": 1})]:
            with self.subTest(name=name):
                result = self.tools.execute(name, arguments, "invalid_" + name)
                self.assertFalse(result["ok"])
        self.assertEqual(self.backend.calls, [])
        self.assertEqual(self.store.state["mission"]["stage"], "IDLE")

    def test_duplicate_call_id_returns_receipt_and_conflicting_reuse_is_rejected(self):
        self.call("set_task", target_color="red", destination="red_slot")
        args = {"cameras": ["d435"], "view": "raw"}
        first = self.tools.execute("observe_scene", args, "same")
        second = self.tools.execute("observe_scene", args, "same")
        self.assertEqual(first, second)
        self.assertEqual(len(self.store.state["observations"]), 1)
        conflict = self.tools.execute("observe_scene", {"cameras": ["astra"]}, "same")
        self.assertEqual(conflict["error"]["code"], "call_id_conflict")

    def test_missing_depth_or_extrinsics_blocks_grasp_generation(self):
        self.observations.geometry = False
        self.call("set_task", target_color="red", destination="red_slot")
        obs = self.call("observe_scene", cameras=["d435"])["data"]["observation_id"]
        self.call("select_target", observation_id=obs, camera="d435", bbox_xyxy=[1, 1, 10, 10], color="red")
        result = self.call("generate_grasp_candidates", observation_id=obs)
        self.assertEqual(result["error"]["code"], "geometry_unavailable")
        self.assertEqual(self.backend.calls, [])

    def test_motion_disabled_blocks_execution_before_backend_submission(self):
        self.prepare_pick()
        self.tools.config.motion_enabled = False
        before = len(self.backend.calls)
        result = self.call("execute_pick", plan_id="pick_1")
        self.assertEqual(result["error"]["code"], "motion_disabled")
        self.assertEqual(len(self.backend.calls), before)
        self.assertIsNone(self.store.state["pending"])

    def test_completed_pick_does_not_authorize_carry_without_confirmed_hold(self):
        self.prepare_pick()
        self.assertTrue(self.call("execute_pick", plan_id="pick_1")["ok"])
        self.assertEqual(self.store.state["mission"]["stage"], "PICK_EXECUTED")
        self.assertFalse(self.call("stow_for_carry")["ok"])
        obs = self.call("observe_scene", cameras=["d435"])["data"]["observation_id"]
        self.backend.hold_status = "unresolved"
        self.assertFalse(self.call("verify_hold", observation_id=obs)["ok"])
        self.assertEqual(self.store.state["mission"]["stage"], "PICK_EXECUTED")

    def test_unknown_execution_survives_reopen_and_is_queried_not_replayed(self):
        self.prepare_pick()
        self.backend.next_status = "unknown"
        result = self.call("execute_pick", plan_id="pick_1")
        self.assertFalse(result["ok"])
        self.assertIsNotNone(self.store.state["pending"])
        self.assertFalse(self.call("execute_pick", plan_id="pick_1")["ok"])
        submitted = len(self.backend.calls)
        path = self.store.path
        self.store.close()
        self.store = self.storage.RunStore.open(path)
        self.addCleanup(self.store.close)
        self.tools = self.tools_module.ToolRegistry(self.config, self.store, self.observations, self.backend)
        result = self.call("get_execution_status")
        self.assertTrue(result["ok"])
        self.assertEqual(len(self.backend.calls), submitted)
        self.assertEqual(self.backend.queries, 1)
        self.assertEqual(self.store.state["mission"]["stage"], "PICK_EXECUTED")
        self.assertIsNone(self.store.state["pending"])

    def test_complete_fixture_workflow_requires_slot_release_and_three_second_evidence(self):
        self.prepare_pick()
        self.assertTrue(self.call("execute_pick", plan_id="pick_1")["ok"])
        obs = self.call("observe_scene", cameras=["d435"])["data"]["observation_id"]
        self.assertTrue(self.call("verify_hold", observation_id=obs)["ok"])
        self.assertTrue(self.call("stow_for_carry")["ok"])
        self.assertTrue(self.call("navigate_to_area")["ok"])
        obs = self.call("observe_scene", cameras=["d435"])["data"]["observation_id"]
        self.assertFalse(self.call("select_slot", observation_id=obs, camera="d435",
                                   bbox_xyxy=[1, 1, 10, 10], slot_id="blue_slot")["ok"])
        self.assertTrue(self.call("select_slot", observation_id=obs, camera="d435",
                                  bbox_xyxy=[1, 1, 10, 10], slot_id="red_slot")["ok"])
        self.assertTrue(self.call("align_battery_to_slot", observation_id=obs)["ok"])
        self.assertTrue(self.call("insert_battery")["ok"])
        self.assertTrue(self.call("release_and_withdraw")["ok"])
        self.assertFalse(self.call("finish_task")["ok"])
        obs = self.call("observe_scene", cameras=["d435"])["data"]["observation_id"]
        self.backend.stable_seconds = 2.9
        self.assertFalse(self.call("verify_placement", observation_id=obs)["ok"])
        self.backend.stable_seconds = 3.0
        self.assertTrue(self.call("verify_placement", observation_id=obs)["ok"])
        self.assertTrue(self.call("finish_task")["ok"])
        self.assertEqual(self.store.state["mission"]["stage"], "COMPLETED")

    def test_bbox_must_belong_to_known_snapshot_and_selected_target_color(self):
        self.call("set_task", target_color="red", destination="red_slot")
        obs = self.call("observe_scene", cameras=["d435"])["data"]["observation_id"]
        for change in [{"bbox_xyxy": [0, 0, 1000, 1000]}, {"color": "blue"},
                       {"observation_id": "../../other"}, {"camera": "astra"}]:
            args = dict(observation_id=obs, camera="d435", bbox_xyxy=[1, 1, 10, 10], color="red")
            self.assertFalse(self.call("select_target", **dict(args, **change))["ok"])
        self.assertEqual(self.store.state["mission"]["stage"], "TASK_ACCEPTED")

    def test_cancel_prevents_next_tool_and_preserves_pending_receipt(self):
        self.prepare_pick()
        self.backend.next_status = "unknown"
        self.call("execute_pick", plan_id="pick_1")
        before = len(self.backend.calls)
        (self.store.path / "cancel.flag").touch()
        result = self.call("stow_for_carry")
        self.assertEqual(result["error"]["code"], "cancelled")
        self.assertEqual(len(self.backend.calls), before)
        self.assertIsNotNone(self.store.state["pending"])

    def test_second_live_writer_cannot_open_same_run(self):
        with self.assertRaises(RuntimeError):
            self.storage.RunStore.open(self.store.path)


class FixtureObservations:
    """Test-only geometry; never exposed as a production camera backend."""
    geometry = True

    def __init__(self, store):
        self.store = store

    def capture(self, cameras, view="raw"):
        return self.store.save_observation({
            "captured_at": time.time(), "source": "TEST_FIXTURE",
            "cameras": {camera: {"image_size_px": [16, 12], "image_file": None,
                                  "geometry_ready": self.geometry, "source_sequence": len(self.store.state["observations"]) + 1}
                        for camera in cameras},
            "cross_camera_synchronized": False, "robot_state_synchronized": False,
            "errors": {},
        }, {})


class FixtureBackend:
    """Deterministic external-service stand-in used only by tests."""
    def __init__(self):
        self.calls = []
        self.queries = 0
        self.next_status = "completed"
        self.hold_status = "confirmed"
        self.stable_seconds = 3.0
        self.last = None

    def available(self, operation):
        return True

    def invoke(self, operation, arguments, request_id):
        self.calls.append((operation, arguments, request_id))
        data = self.payload(operation, arguments)
        self.last = {"status": "completed", "execution_id": request_id, "data": data}
        status, self.next_status = self.next_status, "completed"
        return dict(self.last, status=status)

    def status(self, operation, request_id):
        self.queries += 1
        return self.last

    def payload(self, operation, arguments):
        if operation == "generate_grasp_candidates":
            return {"candidates": [{"candidate_id": "grasp_1", "frame_id": "TEST_BASE", "position_m": [0.1, 0.2, 0.3],
                                     "orientation_xyzw": [0, 0, 0, 1], "jaw_width_m": 0.04}]}
        if operation == "plan_pick":
            return {"plan_id": "pick_1", "ik_valid": True, "collision_checked": True, "collision_free": True}
        if operation == "verify_hold":
            return {"hold_status": self.hold_status, "target_id": arguments["target"]["target_id"],
                    "evidence_observation_id": arguments["observation_id"], "evidence_after_execution": True}
        if operation == "verify_placement":
            return {"placement_status": "confirmed", "slot_id": "red_slot", "correct_slot": True,
                    "inserted": True, "released": True, "stable_for_s": self.stable_seconds,
                    "evidence_observation_id": arguments["observation_id"], "evidence_after_execution": True}
        return {"pose_reached": True, "completed": True, "carry_ready": True,
                "arrived": True, "aligned": True, "inserted": True,
                "released": True, "withdrawn": True}
