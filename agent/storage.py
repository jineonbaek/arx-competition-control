"""One writer per run; durable state and immutable camera artifacts."""
import copy
import datetime
import json
import os
from pathlib import Path
import socket
import time
import uuid

from .contracts import AgentError, strict_json


def pid_alive(pid):
    if type(pid) is not int or pid <= 0:
        raise ValueError('Invalid writer PID')
    if os.name == 'nt':
        # Windows os.kill(pid, 0) can terminate a process; query its handle instead.
        import ctypes
        from ctypes import wintypes
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.OpenProcess.restype = wintypes.HANDLE
        kernel.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
        handle = kernel.OpenProcess(0x1000, False, pid)
        if not handle:
            if ctypes.get_last_error() == 87:
                return False
            raise PermissionError('Cannot query writer process')
        try:
            code = wintypes.DWORD()
            if not kernel.GetExitCodeProcess(handle, ctypes.byref(code)):
                raise PermissionError('Cannot query writer exit code')
            return code.value == 259
        finally:
            kernel.CloseHandle(handle)
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def atomic_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, allow_nan=False, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


class RunStore:
    @classmethod
    def create(cls, root):
        root = Path(root)
        root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = root / ("run-" + stamp + "-" + uuid.uuid4().hex[:8])
        path.mkdir(mode=0o700)
        store = cls(path)
        store.state = {"schema_version": 1, "run_id": path.name, "mission": {"stage": "IDLE", "task": None},
                       "calls": {}, "pending": None, "observations": {}, "messages": [], "outcome": None}
        store.save()
        store.event("run_created", {"run_id": path.name})
        return store

    @classmethod
    def open(cls, path):
        store = cls(Path(path))
        try:
            store.state = strict_json((store.path / "state.json").read_text(encoding="utf-8"))
            if store.state.get("schema_version") != 1:
                raise AgentError("invalid_run", "Unsupported run schema")
        except Exception:
            store.close()
            raise
        return store

    def __init__(self, path):
        self.path = Path(path).resolve()
        self.closed = True
        self.lock_token = uuid.uuid4().hex
        lock = self.path / "writer.lock"
        try:
            fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        except FileExistsError:
            owner = strict_json(lock.read_text(encoding="utf-8"))
            if owner.get("host") != socket.gethostname():
                raise RuntimeError("Run is owned by another host; inspect the writer lock")
            if not pid_alive(owner['pid']):
                lock.unlink()
                fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
            else:
                raise RuntimeError("Run already has a live writer")
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump({"pid": os.getpid(), "host": socket.gethostname(), "token": self.lock_token}, handle)
        self.closed = False

    def save(self):
        if self.closed:
            raise RuntimeError("Run store is closed")
        atomic_json(self.path / "state.json", self.state)

    def event(self, kind, data):
        record = {"time": time.time(), "event": kind, "data": data}
        with (self.path / "events.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            handle.flush()

    def save_observation(self, manifest, images):
        observation = copy.deepcopy(manifest)
        observation_id = "obs_" + uuid.uuid4().hex
        directory = self.path / "observations" / observation_id
        directory.mkdir(parents=True)
        for camera, body in images.items():
            if camera not in ("d435", "d405", "astra") or camera not in observation["cameras"]:
                raise AgentError("invalid_observation", "Unexpected image camera")
            (directory / (camera + ".jpg")).write_bytes(body)
            observation["cameras"][camera]["image_file"] = camera + ".jpg"
        observation.update(observation_id=observation_id, recorded_at=time.time())
        atomic_json(directory / "manifest.json", observation)
        self.state["observations"][observation_id] = str(directory.relative_to(self.path))
        self.save()
        self.event("observation", {"observation_id": observation_id, "cameras": list(observation["cameras"]),
                                    "errors": observation.get("errors", {})})
        return observation

    def observation(self, observation_id):
        relative = self.state["observations"].get(observation_id)
        if relative is None:
            raise AgentError("observation_missing", "Observation is not part of this run")
        directory = (self.path / relative).resolve()
        if not directory.is_relative_to(self.path):
            raise AgentError("invalid_run", "Invalid observation path")
        return strict_json((directory / "manifest.json").read_text(encoding="utf-8"))

    def image_path(self, observation_id, camera):
        observation = self.observation(observation_id)
        frame = observation["cameras"].get(camera)
        if not frame or frame.get("image_file") != camera + ".jpg":
            raise AgentError("image_missing", "No stored image for this camera")
        return self.path / self.state["observations"][observation_id] / frame["image_file"]

    @property
    def cancelled(self):
        return (self.path / "cancel.flag").exists()

    def close(self):
        if not self.closed:
            lock = self.path / "writer.lock"
            if lock.exists() and strict_json(lock.read_text(encoding="utf-8")).get("token") == self.lock_token:
                lock.unlink()
            self.closed = True
