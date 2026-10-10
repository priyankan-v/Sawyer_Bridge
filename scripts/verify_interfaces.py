#!/usr/bin/env python3
"""Validate every generated ROS 2 Python type support and serialized interface."""
import importlib
import json
import sys
from pathlib import Path
from rclpy.serialization import serialize_message, deserialize_message

ROOT = Path(__file__).resolve().parents[1]
manifest = json.loads((ROOT/'docs/interface_manifest.json').read_text())
results = []
for entry in manifest:
    path = Path(entry['ros2'])
    package, kind, name = path.parts[2],path.parts[3],path.stem
    cls = getattr(importlib.import_module(package+'.'+kind),name)
    classes = [cls] if kind == 'msg' else ([cls.Request,cls.Response] if kind == 'srv' else [cls.Goal,cls.Result,cls.Feedback])
    for typ in classes:
        msg = typ()
        decoded = deserialize_message(serialize_message(msg),typ)
        assert decoded == msg
    results.append(entry['ros2'])
print(f'PASS: {len(results)} ROS 2 interfaces load compiled type support and serialize.')
if len(sys.argv)>1:
    Path(sys.argv[1]).write_text(json.dumps({'status':'PASS','interfaces':results},indent=2)+'\n')
