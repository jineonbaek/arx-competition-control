#!/usr/bin/env bash
# Build the checked-out RViz source; never select a historical staging overlay.
set -eo pipefail
cd "$(dirname "$0")/.."
root=$PWD
source sensor_env.sh
src="$root/polarbear_adapt/waypoint_rviz"
digest=$(python3 - "$src" <<'PY'
import hashlib,sys
from pathlib import Path
p=Path(sys.argv[1]);h=hashlib.sha256()
for f in sorted(p.iterdir()):
    if f.is_file() and (f.suffix in ('.cpp','.hpp','.h','.xml') or f.name=='CMakeLists.txt'):
        h.update(f.name.encode()+b'\0'+f.read_bytes()+b'\0')
print(h.hexdigest())
PY
)
release="$root/overlays/waypoint_rviz/$digest"
prefix="$release/install"
valid() {
  [[ -s "$prefix/lib/libarx_waypoint_rviz.so" && -s "$prefix/share/arx_waypoint_rviz/plugins.xml" &&
     -f "$prefix/share/ament_index/resource_index/packages/arx_waypoint_rviz" &&
     "$(cat "$release/source.sha256" 2>/dev/null)" == "$digest" ]]
}
if [[ "${1:-}" == --check ]]; then valid; printf '%s\n' "$prefix"; exit 0; fi
mkdir -p "$root/overlays/waypoint_rviz"
exec 8>"$root/overlays/waypoint_rviz/build.lock"
flock 8
if ! valid; then
  echo "正在构建正式源码的 RViz 插件：$digest" >&2
  mkdir -p "$release"
  cmake -S "$src" -B "$release/build" -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_BUILD_TYPE=Release >&2
  nice -n 10 cmake --build "$release/build" --parallel 1 >&2
  ctest --test-dir "$release/build" --output-on-failure >&2
  cmake --install "$release/build" >&2
  printf '%s\n' "$digest" >"$release/source.sha256"
  valid || { echo '正式 RViz 插件构建不完整；不回退旧插件。' >&2; exit 1; }
fi
printf '%s\n' "$prefix"
