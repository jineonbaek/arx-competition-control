"""Explicit profiles: no credential discovery, model download or service startup."""
from dataclasses import dataclass, field, fields
import os
from pathlib import Path
from urllib.parse import urlsplit

from .contracts import AgentError, strict_json


def load_private_env(path):
    """Read literal values only; dotenv files are never executed as shell scripts."""
    path = Path(path)
    if os.name == 'posix' and path.stat().st_mode & 0o077:
        raise AgentError('private_config_permissions', 'Private configuration must have permissions 600')
    values = {}
    for line in path.read_text(encoding='utf-8-sig').splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        key, separator, value = line.partition('=')
        if not separator or key not in ('DASHSCOPE_API_KEY', 'QWEN_BASE_URL') or key in values:
            raise AgentError('invalid_config', 'Invalid private environment file')
        value = value.strip()
        if not value or any(c.isspace() for c in value):
            raise AgentError('invalid_config', 'Private environment values must be nonempty literal strings')
        values[key] = value
    os.environ.update(values)


def checked_url(value):
    parsed = urlsplit(value)
    if parsed.scheme not in ("http", "https") or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise AgentError("invalid_config", "Service URL must be an HTTP(S) base URL without credentials, query or fragment")
    return value.rstrip("/")


@dataclass
class Config:
    console_url: str = "http://127.0.0.1:9983"
    motion_enabled: bool = False
    max_rounds: int = 40
    max_tool_calls: int = 96
    api_timeout_s: int = 180
    service_timeout_s: int = 10
    max_frame_age_s: float = 3.0
    max_image_bytes: int = 5_000_000
    max_output_tokens: int = 16384
    services: dict = field(default_factory=dict)
    profiles: dict = field(default_factory=lambda: {
        "cloud": {"model": "qwen3.8-flash", "base_url_env": "QWEN_BASE_URL",
                  "api_key_env": "DASHSCOPE_API_KEY", "enable_thinking": True, "stream": True},
        "local": {"model": "Qwen3.5-2B", "base_url": "http://127.0.0.1:8000/v1",
                  "enable_thinking": False, "stream": False},
    })

    def __post_init__(self):
        self.console_url = checked_url(self.console_url)
        if type(self.motion_enabled) is not bool:
            raise AgentError("invalid_config", "motion_enabled must be boolean")
        for key, lower, upper in [("max_rounds", 1, 100), ("max_tool_calls", 1, 200),
                                  ("api_timeout_s", 1, 300), ("service_timeout_s", 1, 120),
                                  ("max_image_bytes", 1024, 20_000_000), ("max_output_tokens", 128, 32768)]:
            value = getattr(self, key)
            if type(value) is not int or not lower <= value <= upper:
                raise AgentError("invalid_config", f"Invalid {key}")
        if type(self.max_frame_age_s) not in (int, float) or not 0 < self.max_frame_age_s <= 10:
            raise AgentError("invalid_config", "max_frame_age_s must be in (0,10]")
        if not isinstance(self.services, dict) or not isinstance(self.profiles, dict):
            raise AgentError("invalid_config", "services and profiles must be objects")

    @classmethod
    def load(cls, path=None):
        data = {} if path is None else strict_json(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict) or set(data) - {f.name for f in fields(cls)}:
            raise AgentError("invalid_config", "Unknown configuration fields")
        return cls(**data)

    def profile(self, name, *, require_credentials=True):
        if name not in self.profiles or not isinstance(self.profiles[name], dict):
            raise AgentError("invalid_config", "Unknown model profile")
        profile = dict(self.profiles[name])
        allowed = {"model", "base_url", "base_url_env", "api_key_env", "enable_thinking", "stream"}
        if set(profile) - allowed or not isinstance(profile.get("model"), str) or not profile["model"]:
            raise AgentError("invalid_config", "Invalid model profile")
        for key in ("enable_thinking", "stream"):
            if key in profile and type(profile[key]) is not bool:
                raise AgentError("invalid_config", f"{key} must be boolean")
        url = os.getenv(profile.get("base_url_env", "")) or profile.get("base_url")
        key_name = profile.get("api_key_env")
        key = os.getenv(key_name, "") if key_name else ""
        if require_credentials and (not url or (key_name and not key)):
            raise AgentError("credentials_missing", "Configure the model endpoint and API-key environment variables first")
        return dict(profile, base_url=checked_url(url) if url else None, api_key=key)
