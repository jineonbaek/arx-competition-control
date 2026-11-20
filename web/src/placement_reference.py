"""Reference recording facade for the placement web API."""
import time

from placement_reference_store import (
    comparison_payload,
    load_active_reference,
    save_reference_record,
    select_active_reference,
)


class PlacementReferenceController:
    def __init__(self, root, capture, *, clock=time.time):
        if not callable(capture) or not callable(clock):
            raise ValueError("Invalid placement reference controller")
        self.root = root
        self.capture = capture
        self.clock = clock

    def _capture(self, include_artifacts):
        try:
            value, artifacts = self.capture(bool(include_artifacts))
            if not isinstance(value, dict) or not isinstance(artifacts, dict):
                raise ValueError("capture returned invalid data")
            return value, artifacts
        except Exception as error:
            return {"capture_error": str(error), "d435": None}, {}

    def action(self, data):
        if data == {"op": "record_current"}:
            now = float(self.clock())
            capture, artifacts = self._capture(True)
            record = save_reference_record(self.root, capture, now=now, artifacts=artifacts)
            select_active_reference(self.root, record["reference_id"], now=now)
            return record
        if isinstance(data, dict) and set(data) == {"op", "reference_id"} and data.get("op") == "select":
            return select_active_reference(self.root, data["reference_id"], now=float(self.clock()))
        raise ValueError("Unsupported placement reference operation")

    def snapshot(self):
        reference = load_active_reference(self.root)
        current, _ = self._capture(False)
        return comparison_payload(reference, current)

