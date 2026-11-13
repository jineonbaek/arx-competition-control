#!/usr/bin/env bash
set -eo pipefail
sensor_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$sensor_root/sensor_env.sh"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 - <<'PY'
import json,subprocess
addresses=json.loads(subprocess.check_output(['ip','-j','addr','show','dev','enp130s0'],text=True))
if not any(x.get('local')=='192.168.1.50' for x in addresses[0]['addr_info']):
 raise SystemExit('Missing sensor receive address 192.168.1.50 on enp130s0; see reports/latest-sensor-check-path.txt. No process started.')
PY
exec ros2 launch "$sensor_root/config/sensors_only.launch.py" "$@"
