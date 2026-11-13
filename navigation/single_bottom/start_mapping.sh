#!/usr/bin/env bash
# Sensor-only start. Caller must first confirm the robot is stationary.
set -eo pipefail
if [[ "${1:-}" != --stationary-confirmed && "${1:-}" != --driver-only ]]; then
  echo 'Confirm robot/lift/arms stationary, then pass --stationary-confirmed. No motion commands are sent.' >&2
  exit 2
fi
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source sensor_env.sh
driver=$(pgrep -u "$UID" -f '[l]ivox_ros_driver2_node .*single_bottom_driver' | head -1 || true)
if [[ "${1:-}" == --driver-only && -n "$driver" ]];then exit 0;fi
if pgrep -f '[f]astlio_mapping' >/dev/null || { [[ -z "$driver" ]] && pgrep -f '[l]ivox_ros_driver2_node' >/dev/null; };then
  echo 'An existing sensor/LIO process is running; inspect it before starting another.' >&2
  exit 3
fi
test -f models/lift2s_bottom_upright_20261105/body_voxels.npz
export ARX_SINGLE_RUN
ARX_SINGLE_RUN=$(mktemp -d "$ROOT/reports/bottom-upright-$(date +%Y%m%d-%H%M%S)-XXXXXXXX")
python3 desktop/lidar_ingress.py apply
if [[ "$1" != --driver-only ]];then
  read -r enabled rear_enabled < <(python3 desktop/lidar_ingress.py status)
  dual=$(python3 -c "import sys;sys.path.insert(0,'fixed_dual');from geometry import enabled;print(int(enabled('.')))")
  if [[ "$dual" == 1 && "$rear_enabled" == 1 ]];then enabled=1;fi
  [[ "$enabled" == 1 ]] || { echo '底盘点云已关闭，不能初始化 LIO。';exit 1; }
fi
if ! ip -4 addr show dev enp130s0 | grep -q '192.168.1.50/24'; then
  sudo -n ip addr add 192.168.1.50/24 dev enp130s0
fi
if ! pgrep -f '[p]tp4l' >/dev/null; then
  nohup sudo -n "$ROOT/deps/linuxptp-private/unpacked/usr/sbin/ptp4l" -f "$ROOT/dual_dynamic/ptp-master.cfg" -m > "$ARX_SINGLE_RUN/ptp.log" 2>&1 < /dev/null &
  sleep 8
fi
if [[ -z "$driver" ]];then
nohup ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -r __node:=single_bottom_driver -p xfer_format:=1 -p multi_topic:=1 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=bottom_livox -p user_config_path:="$ROOT/config/mid360s_dual_capture.json" -r /livox/lidar_192_168_1_3:=/single/lidar_raw -r /livox/imu_192_168_1_3:=/single/imu -r /livox/lidar_192_168_1_164:=/unused_head/lidar -r /livox/imu_192_168_1_164:=/unused_head/imu > "$ARX_SINGLE_RUN/driver.log" 2>&1 < /dev/null &
DRIVER_PID=$!
printf '%s\n' "$DRIVER_PID" > "$ARX_SINGLE_RUN/driver.pid"
sleep 10
else
  DRIVER_PID=$driver
  echo "$DRIVER_PID" > "$ARX_SINGLE_RUN/driver.pid"
fi
if [[ "$1" == --driver-only ]];then exit 0;fi
sudo -n timeout 10 tcpdump -i enp130s0 -s 100 -w "$ARX_SINGLE_RUN/clock-stable.pcap" '((src host 192.168.1.3) or (src host 192.168.1.164)) and (udp dst port 56301 or udp dst port 56401)' > "$ARX_SINGLE_RUN/tcpdump.log" 2>&1 < /dev/null || [[ $? == 124 ]]
python3 dual_dynamic/audit_clock.py "$ARX_SINGLE_RUN/clock-stable.pcap" --output "$ARX_SINGLE_RUN/clock-stable.json"
if ! python3 - <<'PY'
import json,os
d=json.load(open(os.environ['ARX_SINGLE_RUN']+'/clock-stable.json'))
from pathlib import Path
flags=json.loads(Path('config/lidar_streams.json').read_text())
expected={'192.168.1.3:56401:0'}
if flags['bottom']:expected.add('192.168.1.3:56301:1')
if flags['head']:expected.update({'192.168.1.164:56301:1','192.168.1.164:56401:0'})
assert d['ptp_type_all'] and expected<=set(d['streams'])
assert all(v['count']>100 and v['nonincreasing']==0 for v in d['streams'].values())
assert all(v['device_gap_max_ms']<100 for v in d['streams'].values())
PY
then
  if [[ -z "$driver" ]];then kill -INT "$DRIVER_PID";fi
  echo 'Clock audit failed; LIO not started.' >&2
  exit 4
fi
# Require live PTP interaction and current device status, not only time_type=1.
if [[ "$dual" == 1 ]];then
  sudo -n timeout 7 tcpdump -i enp130s0 -s 1500 -w "$ARX_SINGLE_RUN/ptp-status.pcap" 'udp port 319 or udp port 320 or ether proto 0x88f7 or udp port 56201' > "$ARX_SINGLE_RUN/ptp-status.log" 2>&1 || [[ $? == 124 ]]
  python3 fixed_dual/clock_gate.py "$ARX_SINGLE_RUN/ptp-status.pcap" config/lidar_streams.json "$ARX_SINGLE_RUN/ptp-status.json" || { echo 'PTP/status audit failed; LIO not initialized.' >&2;exit 5; }
fi
nohup ros2 launch single_bottom/mapping.launch.py > "$ARX_SINGLE_RUN/mapping.log" 2>&1 < /dev/null &
printf '%s\n' "$!" > "$ARX_SINGLE_RUN/mapping.pid"
printf '%s\n' "$ARX_SINGLE_RUN" > reports/active-bottom-single.txt
echo "Started sensor-only mapping: $ARX_SINGLE_RUN"
echo 'Keep stationary for initialization; verify mapping.log and run single_bottom/check.py before moving.'
