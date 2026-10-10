#!/usr/bin/env python3
"""Verify original source hashes and load/serialize all ROS 1 interfaces."""
import hashlib
import importlib
import io
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
manifest=json.loads((ROOT/'docs/interface_manifest.json').read_text())
count=0
for entry in manifest:
    path=ROOT/entry['source']
    assert hashlib.sha256(path.read_bytes()).hexdigest()==entry['sha256'],path
    package,kind,name=path.parent.parent.name,path.parent.name,path.stem
    if kind=='action':
        classes=[getattr(importlib.import_module(package+'.msg'),name+suffix) for suffix in ('Action','ActionGoal','ActionResult','ActionFeedback','Goal','Result','Feedback')]
    else:
        cls=getattr(importlib.import_module(package+'.'+kind),name)
        classes=[cls] if kind=='msg' else [cls._request_class,cls._response_class]
    for cls in classes:
        instance=cls()
        output=io.BytesIO()
        instance.serialize(output)
        decoded=cls().deserialize(output.getvalue())
        assert decoded==instance,cls
    count+=1
print(f'PASS: {count} ROS 1 source interfaces and generated action messages load and serialize; original sources unchanged.')
