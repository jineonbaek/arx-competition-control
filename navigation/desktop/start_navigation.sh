#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
root=$PWD
mode=${1:-navigation}
case "$mode" in mapping|navigation|mapping-navigation|current-map|new-map|resume-map|saved-map) ;; *) exit 2;; esac
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/arx-navigation-${UID}.lock"
flock -n 9 || { echo '另一项雷达或导航操作正在进行。'; exit 1; }
mkdir -p reports
log=$(mktemp "$root/reports/desktop-$mode-$(date +%Y%m%d-%H%M%S)-XXXXXX.log")
exec > >(tee -a "$log" 9>&-) 2>&1
source sensor_env.sh
source desktop/session.sh
fail(){ echo "$1"; if [[ -n "${DISPLAY:-}" ]];then zenity --error --text="$1
$log" || true;fi;exit 1; }
read -r bottom head < <(python3 desktop/lidar_ingress.py status)
[[ "$bottom" == 1 ]] || fail '底盘点云已关闭。请在「雷达开关」中启用底盘点云后再启动。'
python3 desktop/lidar_ingress.py apply || fail '应用雷达开关失败。'
health() {
  python3 desktop/chain_health.py "$@" "${head_check[@]}"
}
save_current() {
  local result status
  result=$(mktemp "$PWD/reports/desktop-map-result-XXXXXXXX")
  if bash desktop/save_navigation_map.sh "$result" </dev/null >&2; then
    cat "$result"
    status=0
  else
    status=1
  fi
  rm -f -- "$result"
  return "$status"
}
head_check=()
[[ "$head" != 1 ]] || head_check+=(--head)
old=$(session_run || true)
old_mode=''
if session_live "$old"; then old_mode=$(session_mode "$old"); fi
checkpoint=''
saved=''
resume=false
use_live_hint=false
force_new=false
case "$mode" in
  current-map)
    case "$old_mode" in mapping|mapping-navigation|legacy-mapping) ;; *) fail '当前没有可切换的建图会话。';; esac
    session_idle || fail '请先停止路线并停车。'
    health mapping || fail '建图链路异常。请先用建图一键启动恢复。'
    saved=$(save_current) || fail '保存失败，原建图会话仍在。'
    checkpoint=$saved
    mode=navigation
    ;;
  navigation)
    selection=$(zenity --list --title='导航一键启动' --column='运行方式' '使用已保存地图' '边建图边导航' 9>&-) || exit 0
    if [[ "$selection" == '边建图边导航' ]]; then mode=mapping-navigation; fi
    ;;
  resume-map)
    mode=${3:-mapping}
    [[ "$mode" == mapping || "$mode" == mapping-navigation ]] || fail '无效的恢复模式。'
    checkpoint=${2:-}
    [[ -s "$checkpoint" && -s "${checkpoint%.yaml}.pgm" && ! -e "${checkpoint%/*}/INCOMPLETE" ]] ||
      fail '保存的地图不完整。'
    if [[ ! -s "${checkpoint%.yaml}.dual-origin.npz" &&
          ( ! -s "${checkpoint%.yaml}.posegraph" || ! -s "${checkpoint%.yaml}.data" ) ]]; then
      fail '没有可恢复的建图数据。'
    fi
    resume=true
    if [[ "${4:-}" == --live-hint ]]; then use_live_hint=true; fi
    ;;
  saved-map)
    mode=navigation
    saved=${2:-}
    [[ -s "$saved" && -s "${saved%.yaml}.pgm" && ! -e "${saved%/*}/INCOMPLETE" ]] ||
      fail '指定地图不完整。'
    ;;
  new-map)
    mode=mapping
    force_new=true
    ;;
esac
if [[ "$mode" == navigation && -z "$saved" ]];then
  for path in "$root"/maps/*/room.yaml;do
    [[ -s "$path" && ! -e "${path%/*}/INCOMPLETE" && -s "${path%/*}/room.pgm" ]] && maps+=("$path")
  done
  ((${#maps[@]})) || fail '没有可用的已保存地图。'
  saved=$(zenity --list --title='选择导航地图' --width=800 --height=350 --column='地图文件' "${maps[@]}") || exit 0
  [[ -s "$saved" ]] || exit 0
fi
same=false
if ! $force_new && ! $resume && [[ "$old_mode" == "$mode" ]]; then
  if [[ "$mode" != navigation || "$(cat "$old/desktop-map" 2>/dev/null)" == "$saved" ]]; then
    if [[ "$mode" != navigation && ! -e "$old/mapping-engine" ]]; then
      same=false
    elif [[ "$mode" == mapping || "${ARX_EXTERNAL_GOAL_DRIVER:-0}" == 1 || -e "$old/drive-ready" ]]; then
      same=true
    elif [[ "$mode" == mapping-navigation ]]; then
      session_idle || fail '请先停止路线并停车。'
      checkpoint=$(save_current) || fail '无法保存旧图，原会话仍在。'
      resume=true
      use_live_hint=true
    fi
  fi
fi
if $same; then
  reuse_stage=mapping
  [[ "$mode" != navigation ]] || reuse_stage=navigation
  reuse_flags=()
  if [[ -f "$old/mapping-engine" ]] && grep -qx dual-origin "$old/mapping-engine"; then
    reuse_flags+=(--dual-origin)
  fi
  if ! problem=$(health "$reuse_stage" --reuse "${reuse_flags[@]}"); then
    echo "$problem"
    zenity --question --title='建图或导航链路异常' --ok-label='尝试恢复' \
      --cancel-label='保留当前会话' --text="$problem" 9>&- || exit 1
    if [[ "$mode" != navigation ]]; then
      session_idle || fail '请先停止路线并停车。'
      checkpoint=$(save_current) || fail '无法保存旧图，原会话仍在。'
      resume=true
    fi
    same=false
  fi
fi
bootstrap=''
pending=''
cleanup(){
  if [[ -n "$bootstrap" ]]; then
    stop_owned "$bootstrap" 'ros2 launch polarbear_adapt/navigation.launch.py' || true
    python3 desktop/session_processes.py stop "$ARX_ADAPT_RUN" || true
  fi
  if [[ -n "$pending" ]]; then
    stop_owned "$pending" 'ros2 launch polarbear_adapt/navigation.launch.py' || true
    python3 desktop/session_processes.py stop "$ARX_ADAPT_RUN"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if ! $same;then
  if [[ -n "$old_mode" ]]; then session_idle || fail '请先停止路线并停车。'; fi
  if ! $force_new && ! $resume && [[ "$mode" != navigation && -n "$old_mode" && "$mode" != "$old_mode" &&
        -f "$old/mapping-engine" ]] && grep -qx dual-origin "$old/mapping-engine"; then
    checkpoint=$(save_current) || fail '无法保存当前双光心地图，原会话仍在。'
    resume=true
    use_live_hint=true
  fi
  if ! health lio --timeout 5; then
    if [[ -n "$old_mode" && "$old_mode" != navigation && -z "$checkpoint" ]]; then
      checkpoint=$(save_current) || fail '无法保存旧图，原会话仍在。'
      if [[ "$mode" != navigation ]] && ! $force_new; then resume=true; fi
    fi
    session_idle || fail '请先停车。'
    zenity --question --title='融合里程计初始化' \
      --text='请保持整车、升降机构和机械臂静止，然后确认。' 9>&- || exit 1
    if pgrep -f '[f]astlio_mapping' >/dev/null; then
      sensor_run=$(cat reports/active-bottom-single.txt)
      stop_owned "$(cat "$sensor_run/mapping.pid")" 'ros2 launch single_bottom/mapping.launch.py' ||
        fail '未能停止现有融合进程。'
    fi
    bash single_bottom/start_mapping.sh --stationary-confirmed 9>&- ||
      fail '融合初始化失败，请检查雷达网口及 PTP 日志。'
    health lio --timeout 45 || fail '融合里程计未持续更新。'
  fi
  export ARX_DESKTOP_MODE="$mode" ARX_ADAPT_RUN ARX_DESKTOP_MANAGED=1
  ARX_ADAPT_RUN=$(mktemp -d "$root/reports/desktop-$mode-$(date +%Y%m%d-%H%M%S)-XXXXXXXX")
  echo "$mode" >"$ARX_ADAPT_RUN/desktop-mode"
  unset ARX_RESUME_GRAPH ARX_RESUME_SEED ARX_SAVED_MAP ARX_MAP_SEED ARX_OBSTACLE_CHECKPOINT ARX_FAST_SENSOR_NODE ARX_SCAN_BOOTSTRAP
  unset ARX_MAPPING_ENGINE ARX_DUAL_ORIGIN_RESUME
  if [[ "$mode" != navigation ]]; then
    if ! $resume || [[ -s "${checkpoint%.yaml}.dual-origin.npz" ]]; then
      [[ "$head" == 1 ]] || fail '双光心建图需要启用前、后雷达。'
      export ARX_MAPPING_ENGINE=dual-origin
    else
      export ARX_MAPPING_ENGINE=slam-toolbox
    fi
    printf '%s\n' "$ARX_MAPPING_ENGINE" >"$ARX_ADAPT_RUN/mapping-engine"
  fi
  if [[ "$mode" == navigation ]] || $resume; then
    target=${saved:-$checkpoint}
    if ! session_live "$old";then
      python3 polarbear_adapt/prepare.py
      ARX_SCAN_BOOTSTRAP=1 ros2 launch polarbear_adapt/navigation.launch.py >"$ARX_ADAPT_RUN/bootstrap.log" 2>&1 </dev/null 9>&- &
      bootstrap=$!
      echo "$bootstrap" >"$ARX_ADAPT_RUN/launch.pid"
    fi
    export ARX_MAP_SEED="$ARX_ADAPT_RUN/seed.json"
    match_args=()
    if [[ -n "$checkpoint" && "$target" == "$saved" && "$saved" == "$checkpoint" &&
          -s "${saved%/*}/pose_hint.json" ]]; then
      match_args+=(--live-hint "${saved%/*}/pose_hint.json")
    elif $use_live_hint && [[ -s "${target%/*}/pose_hint.json" ]]; then
      match_args+=(--live-hint "${target%/*}/pose_hint.json")
    elif $resume && [[ "$old_mode" == navigation && "$(cat "$old/desktop-map" 2>/dev/null)" == "$target" ]]; then
      if python3 desktop/save_pose_hint.py "$ARX_ADAPT_RUN/resume-hint.json"; then
        match_args+=(--live-hint "$ARX_ADAPT_RUN/resume-hint.json")
      fi
    fi
    python3 polarbear_adapt/room_match.py "$target" "$ARX_MAP_SEED" "${match_args[@]}" &&
      python3 -c 'import json,sys;sys.exit(not json.load(open(sys.argv[1]))["acceptable"])' "$ARX_MAP_SEED" || fail "扫描与地图未可靠对齐。地图：$target"
    if [[ -n "$bootstrap" ]]; then
      stop_owned "$bootstrap" 'ros2 launch polarbear_adapt/navigation.launch.py' &&
        python3 desktop/session_processes.py stop "$ARX_ADAPT_RUN" || fail '采样进程未退出。'
      bootstrap=''
    fi
    if $resume; then
      if [[ "$ARX_MAPPING_ENGINE" == dual-origin ]]; then
        export ARX_DUAL_ORIGIN_RESUME="${checkpoint%.yaml}.dual-origin.npz"
      else
        export ARX_RESUME_GRAPH="${checkpoint%.yaml}" ARX_RESUME_SEED="$ARX_MAP_SEED"
      fi
    else
      export ARX_SAVED_MAP="$saved"
      echo "$saved" >"$ARX_ADAPT_RUN/desktop-map"
    fi
  fi
  if session_live "$old"; then
    if [[ -z "$checkpoint" && ( "$old_mode" == *mapping || "$old_mode" == mapping-navigation ) ]]; then
      checkpoint=$(save_current) || fail '无法保存旧图，原会话仍在。'
    fi
    stop_session "$old" "$checkpoint" || fail '原会话仍在运行，请查看日志。'
  fi
  nohup bash polarbear_adapt/start.sh >"$ARX_ADAPT_RUN/launch.log" 2>&1 </dev/null 9>&- &
  child=$!;pending=$child;echo "$child" >"$ARX_ADAPT_RUN/launch.pid"
  for ((i=0;i<70;i++));do
    kill -0 "$child" 2>/dev/null || break
    if grep -q 'Managed nodes are active' "$ARX_ADAPT_RUN/launch.log";then break;fi
    sleep 1
  done
  ready_stage=mapping
  [[ "$mode" != navigation ]] || ready_stage=navigation
  ready_flags=()
  if [[ "${ARX_MAPPING_ENGINE:-}" == dual-origin ]]; then ready_flags+=(--dual-origin); fi
  if ! grep -q 'Managed nodes are active' "$ARX_ADAPT_RUN/launch.log" ||
     ! health "$ready_stage" --timeout 22 "${ready_flags[@]}"; then
    if ! $resume && [[ -n "$checkpoint" && -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] &&
       zenity --question --title='切换未完成' --ok-label='尝试恢复建图' --cancel-label='保留已保存地图' \
         --text="定位或规划尚未就绪。已保存：$checkpoint" 9>&-; then
      cleanup || fail '测试启动的节点仍在退出，请稍后重试。'
      pending=''
      flock -u 9
      exec 9>&-
      exec bash desktop/start_navigation.sh resume-map "$checkpoint"
    fi
    fail "地图或定位没有就绪。${checkpoint:+已保存：$checkpoint}"
  fi
  echo "$ARX_ADAPT_RUN" >"reports/.active-desktop-$$"
  mv "reports/.active-desktop-$$" reports/active-polarbear-adapt.txt
  if [[ "$mode" != mapping && "${ARX_EXTERNAL_GOAL_DRIVER:-0}" != 1 ]]; then touch "$ARX_ADAPT_RUN/drive-ready"; fi
  pending=''
fi
viewer=$(pgrep -u "$UID" -f '[r]viz2 .*polarbear_adapt/polarbear.rviz' || true)
if [[ -n "$viewer" && "$same" == false ]];then
  for pid in $viewer;do stop_owned "$pid" 'polarbear_adapt/polarbear.rviz' || fail '旧 RViz 窗口未退出。';done
  viewer=''
fi
if [[ -z "$viewer" ]];then
  systemd-run --user --quiet --collect --property=Type=exec \
    --property="StandardOutput=append:$log" --property=StandardError=inherit \
    /usr/bin/bash "$root/polarbear_adapt/view.sh" 9>&- || fail '无法启动 RViz。'
else
  python3 desktop/focus_rviz.py $viewer || true
fi
echo "就绪：$mode"
