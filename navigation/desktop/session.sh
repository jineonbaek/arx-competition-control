# Called with cwd set to the navigation root and sensor_env.sh sourced.
session_run() { cat reports/active-polarbear-adapt.txt 2>/dev/null; }
session_live() {
  [[ -f "$1/launch.pid" ]] || return 1
  local cmd pid
  pid=$(cat "$1/launch.pid")
  cmd=$(tr '\0' ' ' 2>/dev/null <"/proc/$pid/cmdline") || cmd=''
  [[ "$cmd" == *"ros2 launch polarbear_adapt/navigation.launch.py"* ]] ||
    python3 desktop/session_processes.py live "$1"
}
session_mode() {
  if [[ -s "$1/desktop-mode" ]]; then cat "$1/desktop-mode"
  elif grep -q 'map_server.*process started' "$1/launch.log"; then echo navigation
  else echo legacy-mapping; fi
}
session_idle() {
  python3 - <<'PY'
import json,urllib.request,sys
try:
    with urllib.request.urlopen('http://127.0.0.1:9983/api/state',timeout=2) as res:
        state=json.load(res)
except (OSError,ValueError):
    # A running driver may still be trying to communicate with the console.
    import subprocess
    sys.exit(subprocess.run(['pgrep','-f','[p]olarbear_adapt/goal_drive.py'],stdout=subprocess.DEVNULL).returncode==0)
sys.exit(bool(state['chassis_active']))
PY
}
stop_owned() {
  local pid=$1 pattern=$2 cmd
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  [[ -f "/proc/$pid/cmdline" ]] || return 0
  cmd=$(tr '\0' ' ' <"/proc/$pid/cmdline")
  [[ "$cmd" == *"$pattern"* ]] || return 1
  local sig
  for sig in INT TERM;do
    cmd=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null) || return 0
    [[ -z "$cmd" ]] && return 0
    [[ "$cmd" == *"$pattern"* ]] || return 1
    kill -"$sig" "$pid" || return 1
    for ((j=0;j<50;j++));do
      [[ -e /proc/$pid/stat ]] || return 0
      [[ "$(ps -p "$pid" -o stat=)" != Z* ]] || return 0
      sleep .2
    done
  done
  return 1
}
stop_session() {
  local run=$1 mode
  session_live "$run" || return 0
  session_idle || { echo '底盘在运动或控制状态无法确认，请停车后切换。'; return 1; }
  mode=$(session_mode "$run")
  if [[ -z "${2:-}" && ( "$mode" == *mapping || "$mode" == mapping-navigation ) ]];then
    echo '正在将当前建图保存到独立目录'
    bash desktop/save_navigation_map.sh </dev/null || return 1
  fi
  if [[ -s "$run/goal-drive.pid" ]];then
    stop_owned "$(cat "$run/goal-drive.pid")" "polarbear_adapt/goal_drive.py" || return 1
  fi
  stop_owned "$(cat "$run/launch.pid")" 'ros2 launch polarbear_adapt/navigation.launch.py' || return 1
  python3 desktop/session_processes.py stop "$run"
}
