"""Read-only camera-frame and robot-state recorder for manual placement."""

import http.cookiejar
import json
from pathlib import Path
import re
import shutil
import signal
import time
import urllib.request


ROOT = Path("/home/arx/arx-competition-control")
LIVE = ROOT / "web/public/camera-live"
REPORTS = Path("/home/arx/arx-place-hold-20261116/reports")
BASE = "http://127.0.0.1:9983"
MAX_SECONDS = 600.0

running = True


def stop(_signum, _frame):
    global running
    running = False


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

opener = urllib.request.build_opener(
    urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar())
)
opener.open(BASE + "/", timeout=3).read()

folder = REPORTS / ("manual-insert-record-" + time.strftime("%Y%m%d-%H%M%S"))
folder.mkdir(parents=True, exist_ok=False)
for camera in ("d435", "d405"):
    (folder / camera).mkdir()

state_file = (folder / "state.jsonl").open("w")
camera_file = (folder / "camera_frames.jsonl").open("w")
seen = {"d435": None, "d405": None}
counts = {"states": 0, "d435": 0, "d405": 0, "camera_errors": 0, "state_errors": 0}
started_wall = time.time()
started_mono = time.monotonic()
last_progress = started_mono

print(json.dumps({"status": "recording", "folder": str(folder)}), flush=True)

try:
    while running and time.monotonic() - started_mono < MAX_SECONDS:
        now = time.time()
        try:
            state = json.load(opener.open(BASE + "/api/state", timeout=2))
            state_file.write(json.dumps({"recorded_at": now, "state": state}) + "\n")
            state_file.flush()
            counts["states"] += 1
        except Exception as exc:
            counts["state_errors"] += 1
            state_file.write(json.dumps({"recorded_at": now, "error": str(exc)[:200]}) + "\n")
            state_file.flush()

        for camera in ("d435", "d405"):
            try:
                meta = json.loads((LIVE / f"{camera}.json").read_text())
                source_name = meta["raw_image_file"]
                if not re.fullmatch(camera + r"-v-[a-f0-9]+-\d+\.jpg", source_name):
                    raise ValueError("Unexpected live image name")
                identity = (source_name, float(meta["captured_at"]))
                if identity == seen[camera]:
                    continue
                source = LIVE / source_name
                sequence = counts[camera]
                destination = folder / camera / f"{sequence:06d}-{identity[1]:.6f}.jpg"
                shutil.copyfile(source, destination)
                camera_file.write(
                    json.dumps(
                        {
                            "recorded_at": now,
                            "camera": camera,
                            "sequence": sequence,
                            "captured_at": identity[1],
                            "source": source_name,
                            "file": str(destination.relative_to(folder)),
                            "meta": meta,
                        }
                    )
                    + "\n"
                )
                camera_file.flush()
                seen[camera] = identity
                counts[camera] += 1
            except Exception as exc:
                counts["camera_errors"] += 1
                camera_file.write(
                    json.dumps(
                        {"recorded_at": now, "camera": camera, "error": str(exc)[:200]}
                    )
                    + "\n"
                )
                camera_file.flush()

        if time.monotonic() - last_progress >= 5.0:
            print(
                json.dumps(
                    {
                        "status": "recording",
                        "elapsed_s": round(time.monotonic() - started_mono, 1),
                        "counts": counts,
                    }
                ),
                flush=True,
            )
            last_progress = time.monotonic()
        time.sleep(0.05)
finally:
    state_file.close()
    camera_file.close()
    manifest = {
        "status": "stopped",
        "started_at": started_wall,
        "ended_at": time.time(),
        "duration_s": time.monotonic() - started_mono,
        "counts": counts,
        "folder": str(folder),
        "read_only_capture": True,
    }
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest), flush=True)
