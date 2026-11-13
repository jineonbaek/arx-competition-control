#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source sensor_env.sh
source desktop/session.sh
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/arx-navigation-${UID}.lock"
flock -n 9 || { zenity --error --text='另一项导航操作正在进行。';exit 1; }
dual=$(python3 -c "import sys;sys.path.insert(0,'fixed_dual');from geometry import enabled;print(int(enabled('.')))")
read -r bottom head < <(python3 desktop/lidar_ingress.py status)
check(){ if [[ "$1" == 1 ]];then echo TRUE;else echo FALSE;fi; }
items=$(zenity --list --checklist --separator=',' --title='雷达开关' --width=640 --height=300 \
  --text='勾选接收的雷达。底盘 IMU 始终保留；固定双雷达模式下任一路可单独参与；两路均关闭会停止 LIO 和导航。' \
  --column='启用' --column='雷达' "$(check "$bottom")" 底盘 "$(check "$head")" 后部（原头部）) || exit 0
b=0;h=0;[[ "$items" == *底盘* ]] && b=1;[[ "$items" == *后部* ]] && h=1
fail(){ zenity --error --text="$1" || true;exit 1; }
if [[ "$b" == 0 && ( "$dual" != 1 || "$h" == 0 ) ]];then
  session_idle || fail '请先让正在运行的底盘停稳。'
  run=$(session_run || true)
  if session_live "$run";then stop_session "$run" || fail '未能保存或停止导航，雷达状态未更改。';fi
  lio=$(cat reports/active-bottom-single.txt 2>/dev/null || true)
  if [[ -n "$lio" && -s "$lio/mapping.pid" ]];then
    stop_owned "$(cat "$lio/mapping.pid")" 'ros2 launch single_bottom/mapping.launch.py' || fail '底盘 LIO 未正常停止。'
  fi
  if pgrep -u "$UID" -f '[f]astlio_mapping.*bottom_upright_lio' >/dev/null;then fail '发现未受当前会话管理的底盘 LIO。';fi
fi
python3 desktop/lidar_ingress.py set "$b" "$h" || fail '无法更新雷达数据开关。'
# Keep the parked-fusion preference in sync, including cold starts.
tmp=$(mktemp config/.head-enabled.XXXXXXXX);echo "$h" >"$tmp";mv "$tmp" config/head_obstacles.enabled
if ros2 node list --no-daemon --spin-time 1 |grep -qx /pb_stationary_head;then
  value=false;[[ "$h" == 1 ]] && value=true
  result=$(timeout 8 ros2 service call /pb/head_enabled std_srvs/srv/SetBool "{data: $value}") &&
      grep -q 'success=True' <<<"$result" || fail '数据开关已设置，但头部停车避障服务未确认。'
fi
bash single_bottom/start_mapping.sh --driver-only 9>&- || fail '驱动未就绪。'
viewer=$(pgrep -u "$UID" -f '[r]viz2 .*desktop/head_only.rviz' || true)
if [[ "$dual" != 1 && "$b" == 0 && "$h" == 1 ]];then
  if ! pgrep -u "$UID" -f '[p]ython3 .*desktop/head_view.py' >/dev/null;then
    nohup python3 "$PWD/desktop/head_view.py" >reports/head-view.log 2>&1 </dev/null 9>&- &
  fi
  if [[ -z "$viewer" ]];then nohup rviz2 -d "$PWD/desktop/head_only.rviz" >>reports/head-view.log 2>&1 </dev/null 9>&- & fi
else
  for pid in $viewer;do stop_owned "$pid" 'desktop/head_only.rviz' || true;done
  for pid in $(pgrep -u "$UID" -f '[p]ython3 .*desktop/head_view.py' || true);do stop_owned "$pid" 'desktop/head_view.py' || true;done
fi
zenity --info --text="底盘点云 $(check "$b")，后部点云和 IMU $(check "$h")。底盘 IMU 已保留。"
