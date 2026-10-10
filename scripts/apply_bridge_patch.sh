#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT/bridge_ws/src/ros1_bridge"
if git apply --reverse --check "$ROOT/patches/ros1_bridge_sawyer.patch" 2>/dev/null; then
  echo 'Sawyer bridge patch is already applied.'
elif git apply --check "$ROOT/patches/ros1_bridge_sawyer.patch"; then
  git apply "$ROOT/patches/ros1_bridge_sawyer.patch"
else
  echo 'Patch does not match this ros1_bridge checkout; inspect before rebuilding.' >&2
  exit 1
fi
