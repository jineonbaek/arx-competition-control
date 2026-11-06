"""Validated, serial tool execution with durable receipts and mission prerequisites."""
import copy
import hashlib
import json
import re
import uuid

from .backends import BackendRouter
from .contracts import AgentError, object_schema, strict_json, validate
from .mission import Mission, OPERATIONS

TEXT = {"type": "string", "minLength": 1, "maxLength": 128}
CAMERA = {"type": "string", "enum": ["d435", "d405", "astra"]}
COLOR = {"type": "string", "enum": ["red", "blue", "yellow", "green"]}
BOX = {"type": "array", "items": {"type": "integer", "minimum": 0}, "minItems": 4, "maxItems": 4}
SCHEMAS = {
    "get_system_status": object_schema(),
    "set_task": object_schema({"target_color": COLOR, "destination": TEXT}),
    "observe_scene": object_schema({"cameras": {"type": "array", "items": CAMERA, "minItems": 1, "maxItems": 3, "uniqueItems": True},
                                    "view": {"type": "string", "enum": ["raw", "annotated"]}}, ["cameras"]),
    "read_observation": object_schema({"observation_id": TEXT}),
    "select_target": object_schema({"observation_id": TEXT, "camera": CAMERA, "bbox_xyxy": BOX, "color": COLOR}),
    "generate_grasp_candidates": object_schema({"observation_id": TEXT}),
    "plan_pick": object_schema({"candidate_id": TEXT, "arm": {"type": "string", "enum": ["left", "right"]}}),
    "execute_pick": object_schema({"plan_id": TEXT}),
    "verify_hold": object_schema({"observation_id": TEXT}),
    "stow_for_carry": object_schema(), "navigate_to_area": object_schema(),
    "select_slot": object_schema({"observation_id": TEXT, "camera": CAMERA, "bbox_xyxy": BOX, "slot_id": TEXT}),
    "align_battery_to_slot": object_schema({"observation_id": TEXT}),
    "insert_battery": object_schema(), "release_and_withdraw": object_schema(),
    "verify_placement": object_schema({"observation_id": TEXT}),
    "get_execution_status": object_schema(), "finish_task": object_schema(),
}
DESCRIPTIONS = {
    "get_system_status": "Read mission stage, pending receipt and configured capabilities. No hardware commands.",
    "set_task": "Bind the instruction's target color and destination once. Do not guess missing task requirements.",
    "observe_scene": "Freeze existing camera HTTP JPEGs and matching metadata. Raw or annotated view. No camera startup. Existing console exposes no raw depth or robot extrinsics.",
    "read_observation": "Read an immutable observation belonging to this run, with its stored camera images.",
    "select_target": "Select one battery in a stored image using original-image integer [x1,y1,x2,y2] pixels and the requested color. This records selection, not a 3D grasp.",
    "generate_grasp_candidates": "Ask the configured grasp backend for metric candidates from an observation with raw depth and robot extrinsics. No robot movement.",
    "plan_pick": "Ask the configured planner to verify IK and collision constraints for a stored candidate and arm.",
    "execute_pick": "Execute the stored verified pick plan through the configured robot backend. Completion is not retention evidence.",
    "verify_hold": "Ask the configured verifier to assess actual target retention using a post-execution observation. The model cannot supply its own confirmation flag.",
    "stow_for_carry": "Use the robot backend to enter a validated carry posture after confirmed retention.",
    "navigate_to_area": "Ask the navigation backend to reach the task's destination area while carrying the target.",
    "select_slot": "Select the specified destination slot in a stored post-arrival image. Bounding box uses original-image xyxy pixels.",
    "align_battery_to_slot": "Ask the insertion backend to align the held battery with the selected slot using fresh geometry.",
    "insert_battery": "Execute the configured insertion capability after alignment. Requires physical backend and motion enabled.",
    "release_and_withdraw": "Release and withdraw only after confirmed insertion using the configured robot capability.",
    "verify_placement": "Require backend evidence for correct slot, insertion, release and at least 3 seconds stability. No self-reported model success.",
    "get_execution_status": "Query the one pending request's existing receipt. Never resubmit the physical operation.",
    "finish_task": "Complete only after verified placement; does not make unverified stages successful.",
}


class ToolRegistry:
    def __init__(self, config, store, observations=None, backends=None):
        self.config, self.store = config, store
        if observations is None:
            from .observations import ConsoleObservations
            observations = ConsoleObservations(config, store)
        self.observations = observations
        self.backends = backends if backends is not None else BackendRouter(config.services, config.service_timeout_s)

    def capabilities(self):
        return {name: self.backends.available(name) and (not physical or self.config.motion_enabled)
                for name, (_, _, physical) in OPERATIONS.items()}

    def definitions(self, *, available_only=False):
        capabilities = self.capabilities()
        output = []
        for name, schema in SCHEMAS.items():
            available = capabilities.get(name, True)
            if available_only and not available:
                continue
            output.append({"type": "function", "function": {"name": name, "description": DESCRIPTIONS[name], "parameters": schema}})
        return output

    def execute(self, name, arguments, call_id):
        fingerprint = None
        try:
            if not isinstance(call_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", call_id):
                raise AgentError("invalid_call_id", "Invalid tool call identity")
            if name not in SCHEMAS:
                raise AgentError("unknown_tool", "Tool is not registered")
            if isinstance(arguments, str):
                try:
                    arguments = strict_json(arguments)
                except (ValueError, TypeError):
                    raise AgentError("invalid_arguments", "Tool arguments must be strict JSON") from None
            validate(arguments, SCHEMAS[name])
            encoded = json.dumps([name, arguments], sort_keys=True, allow_nan=False)
            fingerprint = hashlib.sha256(encoded.encode()).hexdigest()
            previous = self.store.state["calls"].get(call_id)
            if previous:
                if previous["fingerprint"] != fingerprint:
                    raise AgentError("call_id_conflict", "Call ID already refers to different arguments")
                return copy.deepcopy(previous["result"])
            if self.store.cancelled and name != "get_execution_status":
                raise AgentError("cancelled", "Run cancellation requested; this does not stop a hardware controller")
            if self.store.state["pending"] and name not in ("get_execution_status", "get_system_status", "read_observation", "observe_scene"):
                raise AgentError("execution_pending", "Resolve the pending receipt before dependent operations")
            if self.store.state["mission"]["stage"] == "COMPLETED" and name not in ("get_system_status", "read_observation"):
                raise AgentError("task_finished", "Task is already complete")
            data = self._dispatch(name, arguments, call_id)
            result = {"ok": True, "tool": name, "data": data, "error": None}
        except AgentError as error:
            result = {"ok": False, "tool": name, "data": None,
                      "error": {"code": error.code, "message": error.message, "retryable": error.retryable}}
        if fingerprint and not (call_id in self.store.state["calls"]):
            self.store.state["calls"][call_id] = {"fingerprint": fingerprint, "result": result}
        self.store.save()
        self.store.event("tool_result", {"call_id": call_id, "name": name, "ok": result["ok"],
                                         "error": result["error"], "stage": self.store.state["mission"]["stage"]})
        return result

    def _box(self, arguments):
        observation = self.store.observation(arguments["observation_id"])
        frame = observation["cameras"].get(arguments["camera"])
        if frame is None:
            raise AgentError("camera_missing", "Camera does not belong to the selected observation")
        width, height = frame["image_size_px"]
        x1, y1, x2, y2 = arguments["bbox_xyxy"]
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise AgentError("invalid_bbox", "Bounding box is outside the stored image")
        return observation

    def _dispatch(self, name, arguments, call_id):
        state = self.store.state
        mission = Mission(state["mission"])
        if name == "get_system_status":
            return {"mission": copy.deepcopy(mission.data), "pending": copy.deepcopy(state["pending"]),
                    "capabilities": self.capabilities(), "motion_enabled": self.config.motion_enabled,
                    "cancel_requested": self.store.cancelled, "observation_count": len(state["observations"])}
        if name == "set_task":
            mission.require(("IDLE",))
            mission.data.update(stage="TASK_ACCEPTED", task=dict(arguments), target=None, candidates=[], plan=None, slot=None)
            return copy.deepcopy(mission.data)
        if name == "observe_scene":
            return self.observations.capture(arguments["cameras"], arguments.get("view", "raw"))
        if name == "read_observation":
            return self.store.observation(arguments["observation_id"])
        if name in ("select_target", "select_slot"):
            if name == "select_target":
                mission.require(("TASK_ACCEPTED", "TARGET_IDENTIFIED", "GRASP_CANDIDATE_READY", "PICK_PLANNED"))
                if arguments["color"] != mission.data["task"]["target_color"]:
                    raise AgentError("wrong_target", "Selection color does not match the task")
                self._box(arguments)
                selected = dict(arguments, target_id="target_" + uuid.uuid4().hex[:12])
                mission.data.update(stage="TARGET_IDENTIFIED", target=selected, candidates=[], plan=None)
            else:
                mission.require(("ARRIVED_AT_PLACE_AREA", "SLOT_IDENTIFIED"))
                if arguments["slot_id"] != mission.data["task"]["destination"]:
                    raise AgentError("wrong_destination", "Selected slot differs from the task destination")
                self._box(arguments)
                mission.data.update(stage="SLOT_IDENTIFIED", slot=dict(arguments))
                selected = dict(arguments)
            return selected
        if name == "finish_task":
            mission.require(("PLACEMENT_CONFIRMED",))
            mission.data["stage"] = "COMPLETED"
            return {"task_success": True, "stage": "COMPLETED"}
        if name == "get_execution_status":
            pending = state["pending"]
            if not pending:
                return {"pending": None, "stage": mission.stage}
            receipt = self.backends.status(pending["operation"], pending["request_id"])
            return self._receipt(pending["operation"], pending["arguments"], receipt, physical=True)
        return self._operation(name, arguments, call_id, mission)

    def _operation(self, name, arguments, call_id, mission):
        allowed, _, physical = OPERATIONS[name]
        mission.require(allowed)
        if physical and not self.config.motion_enabled:
            raise AgentError("motion_disabled", "Physical execution is disabled in this configuration")
        if not self.backends.available(name):
            raise AgentError("capability_unavailable", "This domain capability has not been configured or verified")
        if name == "generate_grasp_candidates":
            observation = self.store.observation(arguments["observation_id"])
            camera = mission.data["target"]["camera"]
            if mission.data["target"]["observation_id"] != arguments["observation_id"] or not observation["cameras"].get(camera, {}).get("geometry_ready"):
                raise AgentError("geometry_unavailable", "Selected target needs same-observation raw depth and calibrated robot extrinsics")
        if name == "plan_pick" and arguments["candidate_id"] not in {c["candidate_id"] for c in mission.data["candidates"]}:
            raise AgentError("candidate_missing", "Candidate is not part of the current task")
        if name == "execute_pick" and arguments["plan_id"] != mission.data["plan"]["plan_id"]:
            raise AgentError("plan_missing", "Plan is not the current verified plan")
        if "observation_id" in arguments:
            self.store.observation(arguments["observation_id"])
        payload = dict(arguments, task=copy.deepcopy(mission.data["task"]), target=copy.deepcopy(mission.data.get("target")),
                       slot=copy.deepcopy(mission.data.get("slot")), plan=copy.deepcopy(mission.data.get("plan")))
        request_id = self.store.state["run_id"] + ":" + call_id
        if physical:
            self.store.state["pending"] = {"operation": name, "arguments": payload, "request_id": request_id, "call_id": call_id}
            self.store.save()
        self.store.event("backend_request", {"operation": name, "request_id": request_id, "physical": physical})
        try:
            receipt = self.backends.invoke(name, payload, request_id)
        except Exception as error:
            if physical:
                raise AgentError("execution_unknown", "Execution receipt is unresolved; query its status without replaying") from None
            if isinstance(error, AgentError):
                raise
            raise AgentError("backend_failed", "Domain backend failed") from None
        return self._receipt(name, payload, receipt, physical=physical)

    def _receipt(self, name, arguments, receipt, *, physical):
        status = receipt.get("status") if isinstance(receipt, dict) else None
        if status in ("accepted", "running", "unknown") or status is None:
            raise AgentError("execution_pending" if status != "unknown" else "execution_unknown", "Operation has not produced a completed receipt")
        if status == "rejected":
            if physical:
                self.store.state["pending"] = None
            raise AgentError("backend_rejected", "Backend rejected the operation; inspect its diagnostics")
        if status != "completed":
            raise AgentError("invalid_backend_result", "Unknown execution status")
        data = Mission(self.store.state["mission"]).apply(name, receipt.get("data"), arguments)
        if physical:
            self.store.state["pending"] = None
        return {"execution_status": "completed", "execution_id": receipt.get("execution_id"),
                "stage": self.store.state["mission"]["stage"], **data}
