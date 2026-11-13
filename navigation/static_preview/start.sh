#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source sensor_env.sh
export ARX_PREVIEW_RUN="${ARX_PREVIEW_RUN:-$(cat reports/active-static-preview.txt)}"
mkdir -p "$ARX_PREVIEW_RUN"
if ros2 node list 2>/dev/null | grep -qx /preview_level_bridge; then
  echo "Preview already running; use existing view or stop this preview first." >&2
  exit 1
fi
if ! ros2 node list 2>/dev/null | grep -qx /single/bottom_upright_lio; then
  echo "Bottom upright LIO must already be running; no automatic initialization." >&2
  exit 1
fi
python3 static_preview/prepare.py
python3 static_preview/rviz_config.py
exec ros2 launch static_preview/preview.launch.py
