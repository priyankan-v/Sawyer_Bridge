#!/usr/bin/env python3
"""Fail unless every ordinary Sawyer message and service is in actual executable output."""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
pairs = Path(sys.argv[1]).read_text()
manifest = json.loads((ROOT/'docs/interface_manifest.json').read_text())
missing = []
for entry in manifest:
    path = Path(entry['ros2'])
    package,kind,name = path.parts[2],path.parts[3],path.stem
    if kind == 'action':
        continue
    line = f"'{package}/{kind}/{name}' (ROS 2) <=> '{package}/{name}' (ROS 1)"
    if line not in pairs:
        missing.append(line)
assert "'sensor_msgs/msg/JointState' (ROS 2) <=> 'sensor_msgs/JointState' (ROS 1)" in pairs
if missing:
    raise SystemExit('Missing converters:\n'+'\n'.join(missing))
print('PASS: all 46 Sawyer message types, all 3 services, and standard JointState converters are present.')
