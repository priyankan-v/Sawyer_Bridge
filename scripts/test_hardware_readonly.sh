#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT"
: "${ROS_MASTER_URI:?Set Sawyer master address}"
: "${ROS_IP:?Set workstation wired address}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
unset ROS_HOSTNAME
exec env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 /usr/bin/python3 tests/run_hardware_readonly.py
