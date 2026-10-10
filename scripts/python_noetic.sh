#!/usr/bin/env bash
# catkin's cached environment prepends EmPy 4 again; enforce generator ordering.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
export PYTHONNOUSERSITE=1
export PYTHONPATH="$ROOT/tools/empy3:${PYTHONPATH:-}"
exec "$ROOT/.pixi/envs/noetic/bin/python" "$@"
