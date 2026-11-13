#!/usr/bin/env bash
# Each save reserves its own directory, even on simultaneous clicks.
set -eo pipefail
cd "$(dirname "$0")/.."
source sensor_env.sh
mkdir -p maps
export_dir=$(mktemp -d "$PWD/maps/bottom-room-$(date +%Y%m%d-%H%M%S)-XXXXXX")
touch "$export_dir/INCOMPLETE"
finish() {
  local status=$?
  if ((status)); then echo "导出未完成，请查看 $export_dir 中的日志。"; fi
  if [[ -t 0 ]]; then read -r -p '按回车关闭窗口。' _ || true; fi
}
trap finish EXIT
echo "本次地图目录：$export_dir"
prefix="$export_dir/room"
source desktop/session.sh
run=$(session_run || true)
if [[ -n "$run" && -f "$run/mapping-engine" ]] && grep -qx dual-origin "$run/mapping-engine"; then
  request=$(python3 -c 'import json,sys; print(json.dumps({"map_url":sys.argv[1]}))' "$prefix")
  timeout 45 ros2 service call /pb/dual_origin/save_map nav2_msgs/srv/SaveMap "$request" |
    tee "$export_dir/grid.log"
  grep -q 'result=True' "$export_dir/grid.log"
  test -s "$prefix.dual-origin.npz"
  extra=(room.dual-origin.npz)
else
  timeout 45 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: $prefix}" | tee "$export_dir/serialize.log"
  grep -q 'result=0' "$export_dir/serialize.log"
  test -s "$prefix.posegraph" && test -s "$prefix.data" || exit 1
  timeout 30 ros2 run nav2_map_server map_saver_cli -t /pb/persistent_map -f "$prefix" --fmt pgm --mode trinary \
    --ros-args -p save_map_timeout:=15.0 2>&1 | tee "$export_dir/grid.log"
  extra=(room.posegraph room.data)
fi
test -s "$prefix.yaml" && test -s "$prefix.pgm" || exit 1
python3 desktop/save_pose_hint.py "$export_dir/pose_hint.json" || echo '未取得新鲜车位；地图仍可用，定位时请手动指定初始位姿。'
(
 cd "$export_dir"
 files=(room.yaml room.pgm "${extra[@]}")
 if [[ -f pose_hint.json ]]; then files+=(pose_hint.json); fi
 sha256sum "${files[@]}" > SHA256SUMS
 sha256sum -c SHA256SUMS
)
unlink "$export_dir/INCOMPLETE"
if [[ -n "${1:-}" ]]; then printf '%s\n' "$prefix.yaml" >"$1"; fi
echo "保存成功：$prefix.yaml"
