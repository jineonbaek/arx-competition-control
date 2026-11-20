#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source sensor_env.sh
export ARX_DESKTOP_LAUNCHER="$PWD/desktop/start_navigation.sh"
arx_waypoint_prefix=$(bash desktop/prepare_rviz.sh)
export AMENT_PREFIX_PATH="$arx_waypoint_prefix:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$arx_waypoint_prefix/lib:${LD_LIBRARY_PATH:-}"
unset arx_waypoint_prefix
config="$PWD/polarbear_adapt/polarbear.rviz"
python3 "$PWD/polarbear_adapt/rviz_cloud_display.py" &
rear_cloud_pid=$!
cleanup() {
    kill "$rear_cloud_pid" 2>/dev/null || true
    wait "$rear_cloud_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
view_run=$(cat reports/active-polarbear-adapt.txt 2>/dev/null || true)
while true; do
    if rviz2 -d "$config"; then
        if python3 "$PWD/desktop/close_navigation.py" "$view_run"; then
            break
        else
            status=$?
            [[ "$status" == 10 ]] || exit "$status"
        fi
    else
        exit "$?"
    fi
done
