#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -eq 0 ]]; then set -- --desktop; fi
exec /usr/bin/python3 "$ROOT/scripts/start-console.py" "$@"
