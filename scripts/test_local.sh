#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT"
# An independent master and DDS domain prevent fixtures from reaching hardware.
export ROS_MASTER_URI=http://127.0.0.1:11421 ROS_IP=127.0.0.1 ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
unset ROS_HOSTNAME
mkdir -p docs/results .runtime
exec env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_local.py
