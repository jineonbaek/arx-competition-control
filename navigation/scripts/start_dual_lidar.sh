#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/sensor_env.sh"
mkdir -p "$root/runtime"
# Share the original driver's lock: two SDK processes must never own the UDP ports.
exec 9>"$root/runtime/arm-mapping.lock"
flock -n 9 || { echo 'Stop existing single-lidar mapping before starting dual-lidar LIO'; exit 1; }
python3 - <<'PY'
import json, subprocess
addresses = json.loads(subprocess.check_output(['ip', '-j', 'addr', 'show', 'dev', 'enp130s0']))
if not any(a.get('local') == '192.168.1.50' for a in addresses[0]['addr_info']):
    raise SystemExit('Missing sensor receive address 192.168.1.50; restore interface configuration first')
PY
case "${ARX_LIO_SENSORS:-arm,top}" in
  top) sensor_ips=(192.168.1.164) ;;
  arm) sensor_ips=(192.168.1.3) ;;
  arm,top) sensor_ips=(192.168.1.3 192.168.1.164) ;;
  *) echo 'Invalid ARX_LIO_SENSORS'; exit 1 ;;
esac
for sensor in "${sensor_ips[@]}"; do
  ping -I enp130s0 -c 1 -W 1 "$sensor" > /dev/null || exit 1
done
export ARX_DUAL_RUN="$root/reports/dual-lidar-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$ARX_DUAL_RUN"
printf '%s\n' "$ARX_DUAL_RUN" > "$root/reports/latest-dual-lidar-path.txt"
printf '%s\n' "$$" > "$root/runtime/dual-lidar.pid"
export ROS_LOG_DIR="$ARX_DUAL_RUN/ros"
exec ros2 launch "$root/config/dual_lidar/dual_lidar.launch.py"
