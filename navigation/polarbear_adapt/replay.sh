#!/usr/bin/env bash
# Independent perception replay; never sources recorded TF or actuators.
set -eo pipefail
cd "$(dirname "$0")/.."
source sensor_env.sh
export ROS_DOMAIN_ID=199 ARX_REPLAY=1
: "${ARX_BAG_RUN:?Original capture directory}"
: "${ARX_ADAPT_RUN:?New empty output directory}"
[[ ! -e "$ARX_ADAPT_RUN/replay-output" ]] || exit 1
mkdir -p "$ARX_ADAPT_RUN"
cleanup() {
 for pid in "${play_pid:-}" "${record_pid:-}" "${launch_pid:-}"; do
  [[ -z "$pid" ]] || kill -TERM -- "-$pid" 2>/dev/null || true
 done
 wait || true
}
trap cleanup EXIT INT TERM
setsid bash polarbear_adapt/start.sh > "$ARX_ADAPT_RUN/replay-launch.log" 2>&1 &
launch_pid=$!; echo "$launch_pid" > "$ARX_ADAPT_RUN/replay-launch.pid"
setsid ros2 bag record -o "$ARX_ADAPT_RUN/replay-output" /pb/map /global_costmap/costmap /local_costmap/costmap /pb/scan /pb/mapping_scan /pb/ground /localization/odometry /tf > "$ARX_ADAPT_RUN/replay-record.log" 2>&1 &
record_pid=$!
# Supply odometry before Nav2's activation TF timeout. Sensor topics replay at10Hz sim time.
setsid ros2 bag play "$ARX_BAG_RUN/bag" --delay 3 --rate 3.0 --clock 40 --topics /single/cloud_body /localization/odometry --disable-keyboard-controls > "$ARX_ADAPT_RUN/replay-player.log" 2>&1 &
play_pid=$!;echo "$play_pid" > "$ARX_ADAPT_RUN/replay-player.pid"
# /clock starts at0 then jumps to epoch time; activate only after TF/data exist.
for ((attempt=0;attempt<90;attempt++)); do
 [[ ! -f "$ARX_ADAPT_RUN/bridge-health.json" ]] || break
 kill -0 "$play_pid" 2>/dev/null || exit 2
 sleep 1
done
[[ -f "$ARX_ADAPT_RUN/bridge-health.json" ]] || exit 3
ros2 service call /pb_lifecycle/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}' > "$ARX_ADAPT_RUN/lifecycle-start.log" 2>&1
wait "$play_pid"
play_pid=''
