#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
"$ROOT/scripts/env.sh" bridge cmake -S "$ROOT/tests/native" -B "$ROOT/.runtime/native_build" \
  -DCMAKE_C_COMPILER="$ROOT/.pixi/envs/humble/bin/x86_64-conda-linux-gnu-cc" \
  -DCMAKE_CXX_COMPILER="$ROOT/.pixi/envs/humble/bin/x86_64-conda-linux-gnu-c++"
"$ROOT/scripts/env.sh" bridge cmake --build "$ROOT/.runtime/native_build" --parallel 2
"$ROOT/scripts/env.sh" bridge python "$ROOT/tests/native/link_test.py"
exec "$ROOT/scripts/env.sh" bridge "$ROOT/.runtime/native_build/check_conversion"
