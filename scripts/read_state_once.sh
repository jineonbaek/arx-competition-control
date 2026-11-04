#!/bin/bash
# Finite real-device queries only. No motor SDK construction or driver launch.
set -eo pipefail
root=/home/arx/arx-competition-control
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPYCACHEPREFIX="$root/.pycache"
cd "$root"
python3 - <<'PY_STATUS'
from pathlib import Path
import json,time
p=Path("/home/arx/arx-competition-control/reports/live-readonly-status.json")
p.write_text(json.dumps({"all_feedback_verified":False,"state":"query_in_progress","started_at_ns":time.time_ns()},indent=2))
PY_STATUS
python3 scripts/test_readonly_motor_query.py
python3 scripts/test_readonly_status_query.py
python3 scripts/test_readonly_encos_query.py
strace -f -yy -e trace=write,openat -s 256 -o "$root/reports/readonly-status-query-syscalls.log" python3 "$root/scripts/readonly_status_query.py" > "$root/logs/live-dm-query.log" 2>&1
strace -f -yy -e trace=write,openat -s 256 -o "$root/reports/readonly-encos-query-syscalls.log" python3 "$root/scripts/readonly_encos_query.py" > "$root/logs/live-encos-query.log" 2>&1
python3 "$root/scripts/summarize_readonly_state.py"
