#!/usr/bin/env bash
set -eo pipefail
WEB_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_ROOT="$(dirname -- "$WEB_DIR")"
source "$CONTROL_ROOT/env.sh"
export ROS_DOMAIN_ID="${WEB_ROS_DOMAIN_ID:-191}"
placement_args=()
if [[ -f "$CONTROL_ROOT/config/placement.local.json" ]]; then
  placement_args=(--placement-policy "$CONTROL_ROOT/config/placement.local.json")
fi
exec "$CONTROL_ROOT/.venv/bin/python" "$WEB_DIR/src/server.py" --host "${WEB_HOST:-0.0.0.0}" --port "${WEB_PORT:-9983}" --battery-policy "$CONTROL_ROOT/config/battery-execution.json" "${placement_args[@]}"
