"""Inspect or stop children by the exact ARX launch directory they inherited."""
import os
from pathlib import Path
import signal
import sys
import time


def children(run):
    marker = os.fsencode("ARX_ADAPT_RUN=" + run)
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal() or int(entry.name) == os.getpid():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            if marker in (entry / "environ").read_bytes().split(b"\0"):
                yield int(entry.name)
        except (OSError, ProcessLookupError):
            continue


def main():
    action, run = sys.argv[1:]
    if not Path(run).is_absolute() or not (Path(run) / "launch.pid").is_file():
        return 1
    if action == "live":
        return int(next(children(run), None) is None)
    for sig in (signal.SIGINT, signal.SIGTERM):
        for pid in children(run):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        until = time.monotonic() + 8
        while time.monotonic() < until:
            if next(children(run), None) is None:
                return 0
            time.sleep(.25)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
