"""On-demand recognition worker lifecycle. This module imports no ML runtime."""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
DEMO = Path.home() / "arx-vision-demo/vision/demo"
CACHE = ROOT / "web/hardware/runtime/vision-recognition"
CAMERAS = ("d435", "d405")
LEASE_SECONDS = 18

class VisionPreview:
    def __init__(self, cache=CACHE):
        self.cache = Path(cache)
        self.lock = threading.RLock()
        self.process = None
        self.owner = None
        self.job = None
        self.deadline = 0
        self.closed = threading.Event()
        self.watcher = None

    def _command(self, camera, output):
        python = DEMO / ".venv/bin/python"
        script = DEMO / "stream.py"
        if not python.is_file() or not script.is_file():
            raise ValueError("视觉测试环境尚未安装")
        return [str(python), str(script), "--camera", camera, "--output", str(output)]

    def _refresh(self):
        if self.process is not None and self.process.poll() is not None:
            code = self.process.returncode
            self.process = None
            self.job.update(status="error", message="识别进程已退出，请重新开始", exit_code=code)

    def snapshot(self):
        with self.lock:
            self._refresh()
            if self.job is None:
                return {"status": "idle", "message": "点击开始识别后加载模型", "running": False}
            result = dict(self.job)
            if self.process is not None:
                result["pid"] = self.process.pid
                path = self.cache / self.job["run_id"] / "state.json"
                try:
                    live = json.loads(path.read_text())
                    result.update({k: live[k] for k in (
                        "phase", "sequence", "captured_at", "fps", "count", "qwen", "message"
                    ) if k in live})
                except (OSError, ValueError):
                    pass
            result["running"] = self.process is not None
            return result

    def _renew(self):
        self.deadline = time.monotonic() + LEASE_SECONDS
        path = self.cache / self.job["run_id"] / "lease.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps({"expires_at": time.time() + LEASE_SECONDS}))
        os.replace(temporary, path)

    def action(self, client, data):
        if not isinstance(data, dict) or data.get("op") not in ("start", "stop", "heartbeat"):
            raise ValueError("无效视觉识别请求")
        op = data["op"]
        required = {"op", "camera"} if op == "start" else {"op", "run_id"}
        if set(data) != required:
            raise ValueError("视觉识别请求字段无效")
        with self.lock:
            self._refresh()
            if op == "start":
                camera = data["camera"]
                if camera not in CAMERAS:
                    raise ValueError("仅支持 D435 和 D405")
                if self.closed.is_set():
                    raise ValueError("网页服务正在关闭")
                if self.process is not None:
                    if self.owner != client or camera != self.job["camera"]:
                        raise ValueError("已有一路识别正在运行，请先停止")
                    self._renew()
                    return self.snapshot()
                ident = uuid.uuid4().hex
                output = self.cache / ident
                command = self._command(camera, output)
                output.mkdir(parents=True)
                with (output / "worker.log").open("ab") as log:
                    self.process = subprocess.Popen(command, cwd=DEMO,
                        stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                        start_new_session=True,
                        env=dict(os.environ, OMP_NUM_THREADS="4", OPENBLAS_NUM_THREADS="4"))
                self.job = {"run_id": ident, "camera": camera, "status": "running",
                            "phase": "loading", "sequence": 0, "message": "正在加载模型"}
                self.owner = client
                self._renew()
                if self.watcher is None:
                    self.watcher = threading.Thread(target=self._watch, daemon=True)
                    self.watcher.start()
            else:
                if not self.job or data["run_id"] != self.job["run_id"] or client != self.owner:
                    raise ValueError("识别会话已更新，请刷新页面")
                if op == "heartbeat":
                    self._renew()
                else:
                    self._stop("已停止，模型资源已释放")
            return self.snapshot()

    def _stop(self, message):
        if self.process is not None:
            process = self.process
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            except ProcessLookupError:
                process.wait()
            self.process = None
        if self.job:
            self.job.update(status="stopped", message=message, phase="stopped")

    def _watch(self):
        while not self.closed.wait(1):
            with self.lock:
                self._refresh()
                if self.process is not None and time.monotonic() > self.deadline:
                    self._stop("页面连接已断开，识别已自动停止")

    def frame(self, run_id, *, review=False):
        with self.lock:
            state = self.snapshot()
            if not self.job or run_id != self.job["run_id"]:
                return None
            root = self.cache / run_id
            try:
                meta = json.loads((root / "state.json").read_text())
                if review:
                    name = meta.get("qwen", {}).get("image_file", "")
                else:
                    if not state["running"] or time.time() - meta["captured_at"] > 3:
                        return None
                    name = meta.get("image_file", "")
                path = (root / name).resolve()
                if not name or not path.is_relative_to(root.resolve()):
                    return None
                return meta.get("sequence", 0), path.read_bytes()
            except (OSError, KeyError, ValueError):
                return None

    def close(self):
        self.closed.set()
        with self.lock:
            self._stop("网页服务已关闭")
