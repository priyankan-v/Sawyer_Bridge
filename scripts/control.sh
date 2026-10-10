#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
if [[ " $* " == *' --execute '* ]]; then
  "$ROOT/scripts/env.sh" ros1 python "$ROOT/scripts/preflight.py" --require-ready --output "$ROOT/.runtime/preflight.json"
fi
exec "$ROOT/scripts/env.sh" ros2 ros2 run sawyer_ros2_control command "$@"
