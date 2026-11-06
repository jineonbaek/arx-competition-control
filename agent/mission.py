"""Competition prerequisites and evidence-based stage transitions."""
import math

from .contracts import AgentError

# operation: (allowed source stages, next stage, sends physical commands)
OPERATIONS = {
    "generate_grasp_candidates": (("TARGET_IDENTIFIED", "GRASP_CANDIDATE_READY", "PICK_PLANNED"), "GRASP_CANDIDATE_READY", False),
    "plan_pick": (("GRASP_CANDIDATE_READY", "PICK_PLANNED"), "PICK_PLANNED", False),
    "execute_pick": (("PICK_PLANNED",), "PICK_EXECUTED", True),
    "verify_hold": (("PICK_EXECUTED",), "HOLD_CONFIRMED", False),
    "stow_for_carry": (("HOLD_CONFIRMED",), "CARRY_READY", True),
    "navigate_to_area": (("CARRY_READY",), "ARRIVED_AT_PLACE_AREA", True),
    "align_battery_to_slot": (("SLOT_IDENTIFIED", "ALIGNED"), "ALIGNED", True),
    "insert_battery": (("ALIGNED",), "INSERTED", True),
    "release_and_withdraw": (("INSERTED",), "RELEASED", True),
    "verify_placement": (("RELEASED",), "PLACEMENT_CONFIRMED", False),
}


class Mission:
    def __init__(self, state):
        self.data = state

    @property
    def stage(self):
        return self.data["stage"]

    def require(self, stages):
        if self.stage not in stages:
            raise AgentError("stage_not_allowed", f"Operation is not allowed in stage {self.stage}")

    def apply(self, operation, data, arguments):
        self.require(OPERATIONS[operation][0])
        if not isinstance(data, dict):
            raise AgentError("invalid_backend_result", "Backend data must be an object")
        if operation == "generate_grasp_candidates":
            candidates = data.get("candidates")
            if not isinstance(candidates, list) or not candidates or len(candidates) > 100:
                raise AgentError("no_grasp_candidate", "No bounded candidate set was returned")
            identities = set()
            for candidate in candidates:
                if not isinstance(candidate, dict) or not candidate.get("candidate_id") or not candidate.get("frame_id"):
                    raise AgentError("invalid_backend_result", "Candidate needs an ID and coordinate frame")
                for key, length in [("position_m", 3), ("orientation_xyzw", 4)]:
                    value = candidate.get(key)
                    if not isinstance(value, list) or len(value) != length or any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
                        raise AgentError("invalid_backend_result", "Invalid metric grasp pose")
                if abs(sum(x*x for x in candidate["orientation_xyzw"]) - 1) > .01:
                    raise AgentError("invalid_backend_result", "Grasp orientation must be a unit xyzw quaternion")
                width = candidate.get("jaw_width_m")
                if type(width) not in (int, float) or not math.isfinite(width) or width < 0:
                    raise AgentError("invalid_backend_result", "Invalid metric gripper opening")
                if candidate["candidate_id"] in identities:
                    raise AgentError("invalid_backend_result", "Duplicate candidate ID")
                identities.add(candidate["candidate_id"])
            self.data.update(candidates=candidates, plan=None)
        elif operation == "plan_pick":
            if not isinstance(data.get("plan_id"), str) or not data["plan_id"] or any(data.get(k) is not True for k in ("ik_valid", "collision_checked", "collision_free")):
                raise AgentError("plan_not_verified", "Plan requires explicit IK and collision-check evidence")
            if any(key in data and data[key] != arguments[key] for key in ('candidate_id', 'arm')):
                raise AgentError('plan_identity_conflict', 'Planner result contradicts requested candidate or arm')
            self.data["plan"] = dict(data, candidate_id=arguments["candidate_id"], arm=arguments["arm"])
        elif operation in ("verify_hold", "verify_placement"):
            if data.get("evidence_observation_id") != arguments["observation_id"] or data.get("evidence_after_execution") is not True:
                raise AgentError("evidence_missing", "Verification must identify the supplied post-execution observation")
            if operation == "verify_hold":
                if data.get("hold_status") != "confirmed" or data.get("target_id") != self.data["target"]["target_id"]:
                    if data.get("hold_status") == "contradicted":
                        self.data.update(stage="TASK_ACCEPTED", target=None, candidates=[], plan=None)
                    raise AgentError("hold_not_confirmed", "Target retention is not confirmed")
            else:
                seconds = data.get("stable_for_s")
                if data.get("placement_status") != "confirmed" or data.get("slot_id") != self.data["task"]["destination"] or any(data.get(k) is not True for k in ("correct_slot", "inserted", "released")) or type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds < 3:
                    raise AgentError("placement_not_confirmed", "Correct insertion, release and at least 3 seconds stability are required")
            self.data[operation + "_evidence"] = data
        else:
            evidence = {"execute_pick": ("pose_reached",), "stow_for_carry": ("carry_ready",),
                        "navigate_to_area": ("arrived",), "align_battery_to_slot": ("aligned",),
                        "insert_battery": ("inserted",), "release_and_withdraw": ("released", "withdrawn")}[operation]
            if any(data.get(key) is not True for key in evidence):
                raise AgentError("operation_not_confirmed", "The backend did not confirm the required execution condition")
        self.data["stage"] = OPERATIONS[operation][1]
        return data
