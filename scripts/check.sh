#!/bin/bash
set -eo pipefail
root=/home/arx/arx-competition-control
source "$root/env.sh"
cd "$root"
strace -f -e trace=openat,socket,connect,sendto,sendmsg -o "$root/reports/offline-syscalls.log" python "$root/scripts/offline_check.py" > "$root/logs/offline-check.log" 2>&1
python - <<'PY'
from pathlib import Path
import re,json,subprocess
r=Path('/home/arx/arx-competition-control')
text=(r/'reports/offline-syscalls.log').read_text()
blocked=[line for line in text.splitlines() if re.search(r'socket\(AF_CAN|/dev/(?:arxcan|ttyACM|ttyUSB)',line)]
can=subprocess.check_output(['ip','-details','link','show','type','can'],text=True)
report={'hardware_device_syscalls':blocked,'can_interfaces':can,'passed':not blocked and not can.strip()}
(r/'reports/no-hardware-access.json').write_text(json.dumps(report,indent=2))
print((r/'logs/offline-check.log').read_text())
print('NO_HARDWARE_ACCESS',report['passed'])
raise SystemExit(not report['passed'])
PY
