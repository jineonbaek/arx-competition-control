"""Immutable local records for visual placement pose/image comparisons."""
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time
import uuid


REQUIRED = (
    "navigation_pose",
    "chassis_pose",
    "chassis_velocity",
    "lift_rad",
    "right_joints_rad",
    "d435",
)


def _safe_id(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,95}", value):
        raise ValueError("Invalid reference ID")
    return value


def _atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(handle, "w") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def save_reference_record(root, capture, *, now=None, reference_id=None, artifacts=None):
    if not isinstance(capture, dict):
        raise ValueError("Invalid reference capture")
    timestamp = time.time() if now is None else float(now)
    if not math.isfinite(timestamp):
        raise ValueError("Invalid reference capture time")
    reference_id = _safe_id(reference_id or ("ref-%d-%s" % (int(timestamp * 1000), uuid.uuid4().hex[:8])))
    directory = Path(root) / "records" / reference_id
    try:
        directory.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise ValueError("Reference already exists") from error
    artifacts = {} if artifacts is None else artifacts
    if not isinstance(artifacts, dict):
        raise ValueError("Invalid reference artifacts")
    written = []
    for name, payload in artifacts.items():
        if not isinstance(name, str) or Path(name).name != name or not isinstance(payload, (bytes, bytearray)):
            raise ValueError("Invalid reference artifact")
        (directory / name).write_bytes(bytes(payload))
        written.append(name)
    missing = [key for key in REQUIRED if capture.get(key) is None]
    record = {
        "version": 1,
        "reference_id": reference_id,
        "captured_at": timestamp,
        "status": "complete" if not missing else "partial",
        "missing": missing,
        "artifacts": sorted(written),
        "capture": capture,
    }
    _atomic_json(directory / "record.json", record)
    return record


def select_active_reference(root, reference_id, *, now=None):
    reference_id = _safe_id(reference_id)
    record_path = Path(root) / "records" / reference_id / "record.json"
    if not record_path.is_file():
        raise ValueError("Reference does not exist")
    timestamp = time.time() if now is None else float(now)
    if not math.isfinite(timestamp):
        raise ValueError("Invalid active reference time")
    pointer = {"version": 1, "reference_id": reference_id, "selected_at": timestamp}
    _atomic_json(Path(root) / "active.json", pointer)
    return pointer


def load_active_reference(root):
    try:
        pointer = json.loads((Path(root) / "active.json").read_text())
        reference_id = _safe_id(pointer.get("reference_id"))
        return json.loads((Path(root) / "records" / reference_id / "record.json").read_text())
    except (OSError, json.JSONDecodeError, TypeError, ValueError) as error:
        raise ValueError("No active placement reference") from error


def _image(sensor, prefix=""):
    if not isinstance(sensor, dict) or not isinstance(sensor.get("image_file"), str) or not sensor["image_file"]:
        return {"status": "unavailable", "image_file": None}
    return {"status": "available", "image_file": sensor.get("image_url") or prefix + sensor["image_file"]}


def comparison_payload(reference, current):
    if not isinstance(reference, dict) or not isinstance(current, dict):
        raise ValueError("Invalid placement comparison")
    capture = reference.get("capture") if isinstance(reference.get("capture"), dict) else {}
    reference_id = reference.get("reference_id")
    prefix = "/placement-reference/records/%s/" % reference_id if isinstance(reference_id, str) else ""
    return {
        "reference_id": reference_id,
        "images": {
            "reference_d435": _image(capture.get("d435"), prefix),
            "current_d435": _image(current.get("d435")),
        },
        "reference": capture,
        "current": current,
        "residual": current.get("residual") if isinstance(current.get("residual"), dict) else {},
    }

