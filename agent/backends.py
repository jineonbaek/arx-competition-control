"""Explicit domain-service adapter; no connection to the legacy command endpoint."""
import json
import os
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

from .config import checked_url
from .contracts import AgentError, strict_json

DOMAINS = {"generate_grasp_candidates": "grasp", "plan_pick": "planner",
           "execute_pick": "robot", "verify_hold": "verification", "stow_for_carry": "robot",
           "navigate_to_area": "navigation", "align_battery_to_slot": "insertion",
           "insert_battery": "insertion", "release_and_withdraw": "robot",
           "verify_placement": "verification"}


class BackendRouter:
    def __init__(self, services, timeout_s=10):
        self.services, self.timeout_s = {}, timeout_s
        for domain, spec in services.items():
            if domain not in set(DOMAINS.values()) or not isinstance(spec, dict):
                raise AgentError("invalid_config", "Unknown backend service")
            if set(spec) - {"url", "token_env"}:
                raise AgentError("invalid_config", "Unknown backend configuration field")
            if spec.get("url"):
                self.services[domain] = dict(spec, url=checked_url(spec["url"]))

    def available(self, operation):
        return DOMAINS.get(operation) in self.services

    def invoke(self, operation, arguments, request_id):
        return self._request(operation, "/operations/" + quote(operation, safe=""),
                             {"request_id": request_id, "arguments": arguments})

    def status(self, operation, request_id):
        return self._request(operation, "/executions/" + quote(request_id, safe=""), None)

    def _request(self, operation, route, body):
        spec = self.services.get(DOMAINS.get(operation))
        if not spec:
            raise AgentError("capability_unavailable", "The required domain backend is not configured")
        headers = {"Accept": "application/json"}
        if spec.get("token_env"):
            key = os.getenv(spec["token_env"])
            if not key:
                raise AgentError("backend_credentials_missing", "Backend credential is not configured")
            headers["Authorization"] = "Bearer " + key
        data = None
        if body is not None:
            data = json.dumps(body, allow_nan=False).encode()
            headers["Content-Type"] = "application/json"
        try:
            with urlopen(Request(spec["url"] + route, data=data, headers=headers), timeout=self.timeout_s) as response:
                raw = response.read(2_000_001)
            if len(raw) > 2_000_000:
                raise ValueError("Oversized response")
            result = strict_json(raw)
            if not isinstance(result, dict) or result.get("status") not in ("completed", "accepted", "running", "unknown", "rejected"):
                raise ValueError("Invalid service receipt")
            return result
        except (OSError, URLError, HTTPError, ValueError, KeyError):
            # A transport/protocol failure does not prove a physical request was not executed.
            raise AgentError("execution_unknown", "Backend reply unavailable or invalid; query the existing request receipt") from None
