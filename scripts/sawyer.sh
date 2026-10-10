#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
if [[ -f "$ROOT/config/network.env" ]]; then source "$ROOT/config/network.env"; fi
: "${ROS_MASTER_URI:?Set ROS_MASTER_URI or copy and verify config/network.env.example}"
: "${ROS_IP:?Set ROS_IP to the workstation wired address}"
unset ROS_HOSTNAME
exec "$ROOT/scripts/env.sh" ros1 python "$ROOT/scripts/preflight.py" "$@"
