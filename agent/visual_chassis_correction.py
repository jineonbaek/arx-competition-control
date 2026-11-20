"""Visual residual planning for chassis-only cabinet alignment."""
import json
import math
from pathlib import Path


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("Invalid " + name)
    result = [float(v) for v in value]
    if not all(math.isfinite(v) for v in result):
        raise ValueError("Invalid " + name)
    return result


def _policy(value):
    keys = (
        "observation_max_age_s",
        "admission_forward_m",
        "admission_lateral_m",
        "admission_yaw_deg",
        "max_translation_step_m",
        "max_yaw_step_deg",
        "max_total_translation_m",
        "max_total_yaw_deg",
        "handoff_max_age_s",
    )
    if not isinstance(value, dict) or value.get("version") != 1:
        raise ValueError("Invalid visual chassis policy")
    result = dict(value)
    for key in keys:
        number = value.get(key)
        if not _finite(number) or number <= 0:
            raise ValueError("Invalid visual chassis policy")
        result[key] = float(number)
    cycles = value.get("max_correction_cycles")
    if type(cycles) is not int or not 1 <= cycles <= 50:
        raise ValueError("Invalid visual chassis policy")
    result["max_correction_cycles"] = cycles
    if (
        result["admission_forward_m"] > result["max_translation_step_m"]
        or result["admission_lateral_m"] > result["max_translation_step_m"]
        or result["max_translation_step_m"] > result["max_total_translation_m"]
        or result["admission_yaw_deg"] > result["max_yaw_step_deg"]
        or result["max_yaw_step_deg"] > result["max_total_yaw_deg"]
    ):
        raise ValueError("Invalid visual chassis policy")
    return result


def load_chassis_policy(path):
    try:
        return _policy(json.loads(Path(path).read_text()))
    except (OSError, json.JSONDecodeError, TypeError) as error:
        raise ValueError("Invalid visual chassis policy") from error


def _fresh(sensor, now, max_age):
    return (
        isinstance(sensor, dict)
        and sensor.get("valid") is True
        and _finite(sensor.get("captured_at"))
        and 0 <= now - float(sensor["captured_at"]) <= max_age
    )


def _yaw_deg(reference_axis, current_axis):
    ref = _vector(reference_axis, 3, "D435 reference axis")
    cur = _vector(current_axis, 3, "D435 current axis")
    rn = math.hypot(ref[0], ref[1])
    cn = math.hypot(cur[0], cur[1])
    if rn < 0.9 or cn < 0.9:
        raise ValueError("Invalid D435 cabinet axis")
    ref = [ref[0] / rn, ref[1] / rn]
    cur = [cur[0] / cn, cur[1] / cn]
    return math.degrees(math.atan2(ref[0] * cur[1] - ref[1] * cur[0], ref[0] * cur[0] + ref[1] * cur[1]))


def compare_observations(reference, current, policy, *, now):
    policy = _policy(policy)
    if not _finite(now) or not isinstance(reference, dict) or not isinstance(current, dict):
        raise ValueError("Invalid visual chassis observations")
    ref_d435 = reference.get("d435")
    cur_d435 = current.get("d435")
    if _fresh(ref_d435, now, float("inf")) and _fresh(cur_d435, now, policy["observation_max_age_s"]):
        ref_center = _vector(ref_d435.get("center_base_m"), 3, "D435 reference center")
        cur_center = _vector(cur_d435.get("center_base_m"), 3, "D435 current center")
        return {
            "source": "d435",
            "forward_m": cur_center[0] - ref_center[0],
            "lateral_m": cur_center[1] - ref_center[1],
            "yaw_deg": _yaw_deg(ref_d435.get("inside_axis_base"), cur_d435.get("inside_axis_base")),
            "coarse_only": False,
        }
    raise ValueError("No fresh D435 visual chassis observation")


def plan_chassis_step(residual, policy, *, cumulative_translation_m=(0.0, 0.0), cumulative_yaw_deg=0.0):
    policy = _policy(policy)
    if not isinstance(residual, dict):
        raise ValueError("Invalid visual chassis residual")
    if residual.get("source") != "d435" or residual.get("coarse_only") is not False:
        raise ValueError("Visual chassis correction requires D435")
    forward = residual.get("forward_m")
    lateral = residual.get("lateral_m")
    yaw = residual.get("yaw_deg")
    if not all(_finite(v) for v in (forward, lateral, yaw)):
        raise ValueError("Invalid visual chassis residual")
    if (
        abs(forward) <= policy["admission_forward_m"]
        and abs(lateral) <= policy["admission_lateral_m"]
        and abs(yaw) <= policy["admission_yaw_deg"]
    ):
        return {"action": "lock_admitted", "admitted": True, "source": residual.get("source")}
    total = _vector(cumulative_translation_m, 2, "cumulative chassis translation")
    distance = math.hypot(float(forward), float(lateral))
    scale = min(1.0, policy["max_translation_step_m"] / distance) if distance else 0.0
    step_forward = float(forward) * scale
    step_lateral = float(lateral) * scale
    step_yaw = max(-policy["max_yaw_step_deg"], min(policy["max_yaw_step_deg"], float(yaw)))
    next_total = [total[0] + step_forward, total[1] + step_lateral]
    next_yaw = float(cumulative_yaw_deg) + step_yaw
    if math.hypot(*next_total) > policy["max_total_translation_m"] + 1e-12 or abs(next_yaw) > policy["max_total_yaw_deg"] + 1e-12:
        raise ValueError("Visual chassis correction envelope exceeded")
    return {
        "action": "correct_chassis",
        "admitted": False,
        "source": residual.get("source"),
        "forward_m": step_forward,
        "lateral_m": step_lateral,
        "yaw_deg": step_yaw,
        "next_cumulative_translation_m": next_total,
        "next_cumulative_yaw_deg": next_yaw,
    }


def finish_visual_chassis_workflow(*, admitted, reason, chassis_pose, now):
    if type(admitted) is not bool or not isinstance(reason, str) or not reason or not isinstance(chassis_pose, dict) or not _finite(now):
        raise ValueError("Invalid visual chassis workflow result")
    return {
        "version": 1,
        "status": "complete" if admitted else "complete_degraded",
        "outcome": "locked_admitted" if admitted else "insertion_skipped",
        "next_phase": "arm_insertion" if admitted else "complete",
        "captured_at": float(now),
        "chassis_locked": True,
        "chassis_stopped": True,
        "admission_valid": admitted,
        "insertion_skipped": not admitted,
        "reason": reason,
        "chassis_pose": dict(chassis_pose),
    }


def load_locked_handoff(path, *, now, max_age_s):
    try:
        value = json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError("Invalid visual chassis handoff") from error
    return validate_locked_handoff(value, now=now, max_age_s=max_age_s)


def validate_locked_handoff(value, *, now, max_age_s):
    stamp = value.get("captured_at") if isinstance(value, dict) else None
    if (
        not _finite(now)
        or not _finite(max_age_s)
        or max_age_s <= 0
        or not _finite(stamp)
        or not 0 <= float(now) - float(stamp) <= float(max_age_s)
        or value.get("status") != "complete"
        or value.get("outcome") != "locked_admitted"
        or value.get("chassis_locked") is not True
        or value.get("chassis_stopped") is not True
        or value.get("admission_valid") is not True
    ):
        raise ValueError("Invalid visual chassis handoff")
    return value


def run_visual_chassis_workflow(reference, policy, *, observe, command_chassis, stop_chassis, write_handoff, clock):
    policy = _policy(policy)
    if not all(callable(value) for value in (observe, command_chassis, stop_chassis, write_handoff, clock)):
        raise ValueError("Invalid visual chassis workflow adapters")
    cumulative = [0.0, 0.0]
    cumulative_yaw = 0.0

    def emit(receipt):
        try:
            write_handoff(receipt)
        except Exception as error:
            receipt = dict(receipt)
            receipt["record_error"] = str(error)
        return receipt

    def stopped_pose():
        result = stop_chassis()
        if not isinstance(result, dict) or result.get("stopped") is not True or not isinstance(result.get("pose"), dict):
            raise ValueError("Chassis stop confirmation unavailable")
        return result["pose"]

    try:
        for _ in range(policy["max_correction_cycles"]):
            observation = observe()
            now = float(clock())
            residual = compare_observations(reference, observation, policy, now=now)
            step = plan_chassis_step(
                residual,
                policy,
                cumulative_translation_m=cumulative,
                cumulative_yaw_deg=cumulative_yaw,
            )
            if step["action"] == "lock_admitted":
                pose = stopped_pose()
                return emit(finish_visual_chassis_workflow(
                    admitted=True,
                    reason="visual chassis correction admitted",
                    chassis_pose=pose,
                    now=float(clock()),
                ))
            command_chassis(step)
            cumulative = list(step["next_cumulative_translation_m"])
            cumulative_yaw = float(step["next_cumulative_yaw_deg"])
        raise ValueError("Visual chassis correction cycles exhausted")
    except Exception as error:
        try:
            pose = stopped_pose()
        except Exception as stop_error:
            pose = {"unavailable": True, "stop_error": str(stop_error)}
        return emit(finish_visual_chassis_workflow(
            admitted=False,
            reason=str(error),
            chassis_pose=pose,
            now=float(clock()),
        ))

