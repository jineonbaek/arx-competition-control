#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec "$FP_ROOT/.venv/bin/python" "$FP_ROOT/infer_snapshot.py" "$@"
