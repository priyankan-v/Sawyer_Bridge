#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
commands=false
head_commands=false
while [[ ${1:-} == --* ]]; do
  case $1 in
    --commands) commands=true;;
    --head-commands) head_commands=true;;
    *) echo "Unknown option: $1" >&2; exit 2;;
  esac
  shift
done
[[ $# -le 1 ]] || { echo 'Usage: start_bridge.sh [--commands] [--head-commands] [config]' >&2; exit 2; }
config=${1:-$ROOT/config/sawyer_bridge.yaml}
"$ROOT/scripts/env.sh" ros1 python - "$config" "$commands" "$head_commands" <<'PY'
import sys, yaml, rospy
rospy.init_node('configure_sawyer_bridge', anonymous=True)
with open(sys.argv[1]) as stream:
    config = yaml.safe_load(stream)
if sys.argv[2] == 'true':
    for entry in config['topics']:
        if entry['topic'] in ('/robot/limb/right/joint_command', '/robot/limb/right/joint_command_timeout', '/robot/limb/right/set_speed_ratio'):
            entry['enabled'] = True
if sys.argv[3] == 'true':
    for entry in config['topics']:
        if entry['topic'] in ('/robot/head/head_state', '/robot/head/command_head_pan'):
            entry['enabled'] = True
for key in ('topics', 'services_1_to_2', 'services_2_to_1'):
    rospy.set_param('/sawyer_bridge/' + key, config.get(key, []))
PY
exec "$ROOT/scripts/env.sh" bridge ros2 run ros1_bridge parameter_bridge \
  /sawyer_bridge/topics /sawyer_bridge/services_1_to_2 /sawyer_bridge/services_2_to_1 \
  __name:=sawyer_selective_bridge
