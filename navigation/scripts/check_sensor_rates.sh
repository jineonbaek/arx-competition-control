#!/usr/bin/env bash
set -eo pipefail
sensor_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$sensor_root/sensor_env.sh"
probe="$sensor_root/build-tools/sensor_rate_probe/sensor_rate_probe"
if [ ! -x "$probe" ]; then
  cmake -S "$sensor_root/tools/sensor_rate_probe" -B "$sensor_root/build-tools/sensor_rate_probe" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$sensor_root/build-tools/sensor_rate_probe" --parallel 2
fi
mkdir -p "$sensor_root/reports"
result="$sensor_root/reports/live-sensor-rates-$(date +%Y%m%d-%H%M%S).csv"
"$probe" --ros-args -p duration:="${1:-30.0}" -p output:="$result" -p reliability:=reliable
cat "$result"
